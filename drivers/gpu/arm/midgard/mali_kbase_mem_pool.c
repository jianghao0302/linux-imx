// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2015-2025 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <mali_kbase.h>
#include <linux/mm.h>
#include <linux/migrate.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/spinlock.h>
#include <linux/shrinker.h>
#include <linux/atomic.h>
#include <linux/version.h>
#if KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE
#include <linux/sched/signal.h>
#else
#include <linux/signal.h>
#endif

#define pool_dbg(pool, format, ...)                                                                \
	dev_dbg(pool->kbdev->dev, "%s-pool [%zu/%zu]: " format, "kctx", kbase_mem_pool_size(pool), \
		kbase_mem_pool_max_size(pool), ##__VA_ARGS__)

#define NOT_DIRTY false
#define NOT_RECLAIMED false

/**
 * can_alloc_page() - Check if the current thread can allocate a physical page
 *
 * @pool:                Pointer to the memory pool.
 * @page_owner:          Pointer to the task/process that created the Kbase context
 *                       for which a page needs to be allocated. It can be NULL if
 *                       the page won't be associated with Kbase context.
 *
 * This function checks if the current thread can make a request to kernel to
 * allocate a physical page. If the process that created the context is exiting or
 * is being killed, then there is no point in doing a page allocation.
 *
 * The check done by the function is particularly helpful when the system is running
 * low on memory. When a page is allocated from the context of a kernel thread, OoM
 * killer doesn't consider the kernel thread for killing and kernel keeps retrying
 * to allocate the page as long as the OoM killer is able to kill processes.
 * The check allows to quickly exit the page allocation loop once OoM killer has
 * initiated the killing of @page_owner, thereby unblocking the context termination
 * for @page_owner and freeing of GPU memory allocated by it. This helps in
 * preventing the kernel panic and also limits the number of innocent processes
 * that get killed.
 *
 * Return: true if the page can be allocated otherwise false.
 */
static inline bool can_alloc_page(struct kbase_mem_pool *pool, struct task_struct *page_owner)
{
	if (page_owner && ((page_owner->flags & PF_EXITING) || fatal_signal_pending(page_owner))) {
		dev_info(pool->kbdev->dev, "%s : Process %s/%d exiting", __func__, page_owner->comm,
			 task_pid_nr(page_owner));
		return false;
	}

	return true;
}

static size_t kbase_mem_pool_capacity(struct kbase_mem_pool *pool)
{
	ssize_t max_size = (ssize_t)kbase_mem_pool_max_size(pool);
	ssize_t cur_size = (ssize_t)kbase_mem_pool_size(pool);

	return max(max_size - cur_size, (ssize_t)0);
}

static bool kbase_mem_pool_is_full(struct kbase_mem_pool *pool)
{
	return kbase_mem_pool_size(pool) >= kbase_mem_pool_max_size(pool);
}

static bool kbase_mem_pool_is_empty(struct kbase_mem_pool *pool)
{
	return kbase_mem_pool_size(pool) == 0;
}

static bool set_pool_new_page_metadata(struct kbase_mem_pool *pool, struct page *p,
				       struct list_head *page_list, size_t *list_size)
{
	struct kbase_page_metadata *page_md = kbase_page_private(p);
	bool not_movable = false;

	lockdep_assert_held(&pool->pool_lock);

	/* Free the page instead of adding it to the pool if it's not movable.
	 * Only update page status and add the page to the memory pool if
	 * it is not isolated.
	 */
	if (!kbase_is_page_migration_enabled())
		not_movable = true;
	else {
		spin_lock(&page_md->migrate_lock);
		if (PAGE_STATUS_GET(page_md->status) == (u8)NOT_MOVABLE) {
			not_movable = true;
		} else if (!WARN_ON_ONCE(IS_PAGE_ISOLATED(page_md->status))) {
			page_md->status = PAGE_STATUS_SET(page_md->status, (u8)MEM_POOL);
			page_md->data.mem_pool.pool = pool;
			page_md->data.mem_pool.kbdev = pool->kbdev;
			list_add(&p->lru, page_list);
			(*list_size)++;
		}
		spin_unlock(&page_md->migrate_lock);
	}

	if (not_movable) {
		kbase_free_page_later(pool->kbdev, p);
		pool_dbg(pool, "skipping a not movable page\n");
	}

	return not_movable;
}

static void kbase_mem_pool_add_locked(struct kbase_mem_pool *pool, struct page *p)
{
	bool queue_work_to_free = false;

	lockdep_assert_held(&pool->pool_lock);

	if (!pool->order && kbase_is_page_migration_enabled()) {
		if (set_pool_new_page_metadata(pool, p, &pool->page_list, &pool->cur_size))
			queue_work_to_free = true;
	} else {
		list_add(&p->lru, &pool->page_list);
		pool->cur_size++;
	}

	if (queue_work_to_free) {
		struct kbase_mem_migrate *mem_migrate = &pool->kbdev->mem_migrate;

		queue_work(mem_migrate->free_pages_workq, &mem_migrate->free_pages_work);
	}

	pool_dbg(pool, "added page\n");
}

static void kbase_mem_pool_add_list_locked(struct kbase_mem_pool *pool, struct list_head *page_list,
					   size_t nr_pages)
{
	bool queue_work_to_free = false;

	lockdep_assert_held(&pool->pool_lock);

	if (!pool->order && kbase_is_page_migration_enabled()) {
		struct page *p, *tmp;

		list_for_each_entry_safe(p, tmp, page_list, lru) {
			list_del_init(&p->lru);
			if (set_pool_new_page_metadata(pool, p, &pool->page_list, &pool->cur_size))
				queue_work_to_free = true;
		}
	} else {
		list_splice(page_list, &pool->page_list);
		pool->cur_size += nr_pages;
	}

	if (queue_work_to_free) {
		struct kbase_mem_migrate *mem_migrate = &pool->kbdev->mem_migrate;

		queue_work(mem_migrate->free_pages_workq, &mem_migrate->free_pages_work);
	}

	pool_dbg(pool, "added %zu pages\n", nr_pages);
}

static void kbase_mem_pool_add_list(struct kbase_mem_pool *pool, struct list_head *page_list,
				    size_t nr_pages)
{
	kbase_mem_pool_lock(pool);
	kbase_mem_pool_add_list_locked(pool, page_list, nr_pages);
	kbase_mem_pool_unlock(pool);
}

static void kbase_mem_pool_add(struct kbase_mem_pool *pool, struct page *p)
{
	kbase_mem_pool_lock(pool);
	kbase_mem_pool_add_locked(pool, p);
	kbase_mem_pool_unlock(pool);
}

static void kbase_mem_pool_sync_page(struct kbase_mem_pool *pool, struct page *p)
{
	struct device *dev = pool->kbdev->dev;
	dma_addr_t dma_addr = pool->order ? kbase_dma_addr_as_priv(p) : kbase_dma_addr(p);

	dma_sync_single_for_device(dev, dma_addr, (PAGE_SIZE << pool->order), DMA_BIDIRECTIONAL);
}

static void kbase_mem_pool_zero_page(struct kbase_mem_pool *pool, struct page *p)
{
	uint i;

	for (i = 0; i < (1U << pool->order); i++)
		clear_highpage(p + i);

	kbase_mem_pool_sync_page(pool, p);
}

/* Return true if linked, otherwise false */
static bool is_pool_linked_to_pages_defer_ctrl(struct kbase_mem_pool *pool)
{
	lockdep_assert_held(&pool->pool_lock);

	return !list_empty(&pool->link_to_ctrl);
}

/**
 * kbase_mem_pool_free_pages_from_defer_list_locked() - Free pages from deferred_pages_list
 *                                                      Caller must hold the pool lock
 *
 * @pool: Pointer to the memory pool.
 * @from_defer_ctrl: Indicating the caller is defer_controller.
 *
 * This function depend on pool capacity,
 * Function pop pages from deferred list and
 *  - free pages to kernel
 *  - or add them to free_pages list
 *  - or do both.
 *
 */
static void kbase_mem_pool_free_pages_from_defer_list_locked(struct kbase_mem_pool *pool,
							     bool from_defer_ctrl)
{
	int nr_to_pool;
	int nr_to_kernel;
	int deferred_size;
	LIST_HEAD(free_page_list);
	struct page *p, *tmp;

	lockdep_assert_held(&pool->pool_lock);
	/* If the pool is hooked on a defer_ctrl list, check if deferral window is passed */
	if (is_pool_linked_to_pages_defer_ctrl(pool)) {
		if (!is_csf_scheduler_protm_seq_completed(pool->kbdev,
							  atomic_read(&pool->defer_seq)))
			return;
		/* Defer completed, remove pool from defer_ctrl list */
		kbase_csf_scheduler_pages_defer_ctrl_drop_pool(pool, from_defer_ctrl);
	}

	deferred_size = atomic_read(&pool->deferred_size);
	if (!deferred_size)
		return;
	nr_to_pool = kbase_mem_pool_capacity(pool);
	nr_to_pool = min(deferred_size, nr_to_pool);
	nr_to_kernel = deferred_size > nr_to_pool ? deferred_size - nr_to_pool : 0;

	list_for_each_entry_safe(p, tmp, &pool->deferred_pages_list, lru) {
		list_del_init(&p->lru);
		if (nr_to_kernel) {
			nr_to_kernel--;
			if (!pool->order && kbase_is_page_migration_enabled()) {
				kbase_free_page_later(pool->kbdev, p);
				pool_dbg(pool, "deferred page to be freed to kernel later\n");
			} else {
				uint i;
				dma_addr_t dma_addr = kbase_dma_addr_as_priv(p);

				for (i = 0; i < (1u << pool->order); i++)
					kbase_clear_dma_addr_as_priv(p + i);

				dma_unmap_page(pool->kbdev->dev, dma_addr,
					       (PAGE_SIZE << pool->order), DMA_BIDIRECTIONAL);

				pool->kbdev->mgm_dev->ops.mgm_free_page(
					pool->kbdev->mgm_dev, pool->group_id, p, pool->order);
				pool_dbg(pool, "freed deferred page to kernel\n");
			}
		} else {
			list_add(&p->lru, &free_page_list);
			pool_dbg(pool, "move deferred page to free page list\n");
		}
	}

	if (nr_to_pool) {
		/* add rest of deferred pages to free pages list */
		kbase_mem_pool_add_list_locked(pool, &free_page_list, nr_to_pool);
	}

	atomic_set(&pool->deferred_size, 0);
}

void kbase_mem_pool_free_pages_from_deferred_list(struct kbase_mem_pool *pool, bool from_defer_ctrl)
{
	kbase_mem_pool_lock(pool);
	/* If the pool is dying, leave the action to be done by pool_term call */
	if (!pool->dying)
		kbase_mem_pool_free_pages_from_defer_list_locked(pool, from_defer_ctrl);
	kbase_mem_pool_unlock(pool);
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_free_pages_from_deferred_list);

/**
 * kbase_mem_pool_deferred_list_size() - get size of deferred page list
 *
 * @pool: Pointer to the memory pool.
 *
 * This function return number of pages stored in deferred pages list
 *
 * Return: size of deferred page list
 */
size_t kbase_mem_pool_deferred_list_size(struct kbase_mem_pool *pool)
{
	return (size_t)atomic_read(&pool->deferred_size);
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_deferred_list_size);

/**
 * kbase_mem_pool_add_deferred_if_required_locked() - Add page to deferre_page list
 *                                                    Caller must hold the pool lock
 *
 * @pool: Pointer to the memory pool.
 * @p:    Pointer to page structure
 *
 * This function check if conditions to move page to deferral
 * instead of returning it to free_pool or to kernel are meet.
 * It it is true page is added to deferred_pages list
 * This function also check if previouse deferral window is passed
 * and if it is, move all pages on deferred list to
 * free_pages list ot to kernel, before adding page p to the
 * deferred list.
 *
 * Return: true if page was added to deferred_pages list
 *         otherwise false
 */
static bool kbase_mem_pool_add_deferred_if_required_locked(struct kbase_mem_pool *pool,
							   struct page *p)
{
	lockdep_assert_held(&pool->pool_lock);
	/* remove pages from deferred list if page defered is completed */
	if (!pool->dying)
		kbase_mem_pool_free_pages_from_defer_list_locked(pool, false);

	/* check if page deferral is required */
	if (kbase_mem_is_pmode_deferral_required(pool->kbdev)) {
		atomic_set(&pool->defer_seq, kbase_csf_scheduler_get_protm_seq_num(pool->kbdev));
		list_add(&p->lru, &pool->deferred_pages_list);
		atomic_add(1, &pool->deferred_size);
		kbase_csf_scheduler_pages_defer_ctrl_add_pool(pool);
		return true;
	}
	return false;
}

/**
 * kbase_mem_pool_add_deferred_if_required() - Add page to deferre_page list
 *
 * @pool: Pointer to the memory pool.
 * @p:    Pointer to page structure
 *
 * This function check if conditions to move page to qarantine
 * instead of returning it to free_pool or to kernel are meet.
 * It it is true page is added to deferred_pages list
 * This function also check if previouse deferral window is passed
 * and if it is, move all pages in deferred list to the
 * free_pages list or to kernel, before adding page p to the
 * deferred list.
 *
 * Return: true if page was added to deferred_pages list
 *         otherwise false
 */
static bool kbase_mem_pool_add_deferred_if_required(struct kbase_mem_pool *pool, struct page *p)
{
	bool ret_val;

	kbase_mem_pool_lock(pool);
	ret_val = kbase_mem_pool_add_deferred_if_required_locked(pool, p);
	kbase_mem_pool_unlock(pool);
	return ret_val;
}

/**
 * kbase_mem_pool_add_array_deferred_locked() - add page array to defere_page_list
 *                                              Caller must hold the pool lock
 *
 * @pool:      Pointer to the memory pool.
 * @nr_pages:  Number of entry in array
 * @pages:     Pointer to array of tagged address
 * @zero:      Flag to zeore pages before add to list
 * @sync:	   Flag to sync cahce before add page to list
 *
 * This function add array of pages to deferred_pages list
 * If zero flag is set, clear page
 * If sync flag is set, sync page
 */
static void kbase_mem_pool_add_array_deferred_locked(struct kbase_mem_pool *pool, size_t nr_pages,
						     struct tagged_addr *pages, bool zero,
						     bool sync)
{
	struct page *p;
	size_t nr_to_pool = 0;
	size_t i;
	LIST_HEAD(new_page_list);

	lockdep_assert_held(&pool->pool_lock);
	/* free pages form deferred list if page defered is completed */
	if (!pool->dying)
		kbase_mem_pool_free_pages_from_defer_list_locked(pool, false);

	if (unlikely(!nr_pages))
		return;

	pool_dbg(pool, "add_array_deferred_locked(%zu, zero=%d, sync=%d):\n", nr_pages, zero, sync);

	/* Zero/sync pages first */
	for (i = 0; i < nr_pages; i++) {
		if (unlikely(!is_valid_addr(pages[i])))
			continue;

		if (is_huge_head(pages[i]) || !is_huge(pages[i])) {
			p = as_page(pages[i]);
			if (zero)
				kbase_mem_pool_zero_page(pool, p);
			else if (sync)
				kbase_mem_pool_sync_page(pool, p);

			list_add(&p->lru, &new_page_list);
			nr_to_pool++;
		}
		pages[i] = as_tagged(KBASE_INVALID_PHYSICAL_ADDRESS);
	}

	if (likely(nr_to_pool)) {
		atomic_set(&pool->defer_seq, kbase_csf_scheduler_get_protm_seq_num(pool->kbdev));
		list_splice(&new_page_list, &pool->deferred_pages_list);
		atomic_add(nr_to_pool, &pool->deferred_size);
		kbase_csf_scheduler_pages_defer_ctrl_add_pool(pool);
	}

	pool_dbg(pool, "add_array_deferred_locked(%zu) added %zu pages to deferred page list\n",
		 nr_pages, nr_to_pool);
}

/**
 * kbase_mem_pool_add_array_deferred() - add page array to defere_page_list
 *
 * @pool:      Pointer to the memory pool.
 * @nr_pages:  Number of entry in array
 * @pages:     Pointer to array of tagged address
 * @zero:      Flag to zeore pages before add to list
 * @sync:	   Flag to sync cahce before add page to list
 *
 * This function add array of pages to deferred_pages list
 * If zero flag is set, clear page
 * If sync flag is set, sync page
 */
static void kbase_mem_pool_add_array_deferred(struct kbase_mem_pool *pool, size_t nr_pages,
					      struct tagged_addr *pages, bool zero, bool sync)
{
	struct page *p;
	size_t nr_to_pool = 0;
	size_t i;
	LIST_HEAD(new_page_list);

	/* free pages from deferred list if pages deferral window is passed */
	kbase_mem_pool_free_pages_from_deferred_list(pool, false);

	if (unlikely(!nr_pages))
		return;

	pool_dbg(pool, "%s(%zu, zero=%d, sync=%d):\n", __func__, nr_pages, zero, sync);

	/* Zero/sync pages first without holding the pool lock */
	for (i = 0; i < nr_pages; i++) {
		if (unlikely(!is_valid_addr(pages[i])))
			continue;

		if (is_huge_head(pages[i]) || !is_huge(pages[i])) {
			p = as_page(pages[i]);
			if (zero)
				kbase_mem_pool_zero_page(pool, p);
			else if (sync)
				kbase_mem_pool_sync_page(pool, p);

			list_add(&p->lru, &new_page_list);
			nr_to_pool++;
		}
		pages[i] = as_tagged(KBASE_INVALID_PHYSICAL_ADDRESS);
	}

	if (likely(nr_to_pool)) {
		kbase_mem_pool_lock(pool);
		atomic_set(&pool->defer_seq, kbase_csf_scheduler_get_protm_seq_num(pool->kbdev));
		list_splice(&new_page_list, &pool->deferred_pages_list);
		atomic_add(nr_to_pool, &pool->deferred_size);
		kbase_csf_scheduler_pages_defer_ctrl_add_pool(pool);
		kbase_mem_pool_unlock(pool);
	}

	pool_dbg(pool, "%s(%zu) added %zu pages to deferred page list\n", __func__, nr_pages,
		 nr_to_pool);
}

static struct page *kbase_mem_pool_remove_locked(struct kbase_mem_pool *pool,
						 enum kbase_page_status status)
{
	struct page *p;

	lockdep_assert_held(&pool->pool_lock);

	if (kbase_mem_pool_is_empty(pool))
		return NULL;

	p = list_first_entry(&pool->page_list, struct page, lru);

	if (!pool->order && kbase_is_page_migration_enabled()) {
		struct kbase_page_metadata *page_md = kbase_page_private(p);

		spin_lock(&page_md->migrate_lock);
		WARN_ON(PAGE_STATUS_GET(page_md->status) != (u8)MEM_POOL);
		page_md->status = PAGE_STATUS_SET(page_md->status, (u8)status);
		spin_unlock(&page_md->migrate_lock);
	}

	list_del_init(&p->lru);
	pool->cur_size--;

	pool_dbg(pool, "removed page\n");

	return p;
}

static struct page *kbase_mem_pool_remove(struct kbase_mem_pool *pool,
					  enum kbase_page_status status)
{
	struct page *p;

	kbase_mem_pool_lock(pool);
	p = kbase_mem_pool_remove_locked(pool, status);
	kbase_mem_pool_unlock(pool);

	return p;
}

struct page *kbase_mem_alloc_page(struct kbase_mem_pool *pool)
{
	struct page *p;
	gfp_t gfp = __GFP_ZERO;
	struct kbase_device *const kbdev = pool->kbdev;
	struct device *const dev = kbdev->dev;
	dma_addr_t dma_addr;
	uint i;

	/* don't warn on higher order failures */
	if (pool->order)
		gfp |= GFP_HIGHUSER | __GFP_NOWARN;
	else
		gfp |= kbase_is_page_migration_enabled() ? GFP_HIGHUSER_MOVABLE : GFP_HIGHUSER;

	p = kbdev->mgm_dev->ops.mgm_alloc_page(kbdev->mgm_dev, pool->group_id, gfp, pool->order);
	if (!p)
		return NULL;

	dma_addr = dma_map_page(dev, p, 0, (PAGE_SIZE << pool->order), DMA_BIDIRECTIONAL);

	if (dma_mapping_error(dev, dma_addr)) {
		kbdev->mgm_dev->ops.mgm_free_page(kbdev->mgm_dev, pool->group_id, p, pool->order);
		return NULL;
	}

	/* Setup page metadata for small pages when page migration is enabled */
	if (!pool->order && kbase_is_page_migration_enabled()) {
		INIT_LIST_HEAD(&p->lru);
		if (!kbase_alloc_page_metadata(kbdev, p, dma_addr, pool->group_id)) {
			dma_unmap_page(dev, dma_addr, PAGE_SIZE, DMA_BIDIRECTIONAL);
			kbdev->mgm_dev->ops.mgm_free_page(kbdev->mgm_dev, pool->group_id, p,
							  pool->order);
			return NULL;
		}
	} else {
		WARN_ON(dma_addr != page_to_phys(p));
		for (i = 0; i < (1u << pool->order); i++)
			kbase_set_dma_addr_as_priv(p + i, dma_addr + PAGE_SIZE * i);
	}

	return p;
}
KBASE_EXPORT_TEST_API(kbase_mem_alloc_page);

static void enqueue_free_pool_pages_work(struct kbase_mem_pool *pool)
{
	if (!pool->order && kbase_is_page_migration_enabled()) {
		struct kbase_mem_migrate *mem_migrate = &pool->kbdev->mem_migrate;

		queue_work(mem_migrate->free_pages_workq, &mem_migrate->free_pages_work);
	}
}

void kbase_mem_pool_free_page(struct kbase_mem_pool *pool, struct page *p)
{
	struct kbase_device *kbdev;

	if (WARN_ON(!pool))
		return;
	if (WARN_ON(!p))
		return;

	kbdev = pool->kbdev;

	if (!pool->order && kbase_is_page_migration_enabled()) {
		kbase_free_page_later(kbdev, p);
		pool_dbg(pool, "page to be freed to kernel later\n");
	} else {
		uint i;
		dma_addr_t dma_addr = kbase_dma_addr_as_priv(p);

		for (i = 0; i < (1u << pool->order); i++)
			kbase_clear_dma_addr_as_priv(p + i);

		dma_unmap_page(kbdev->dev, dma_addr, (PAGE_SIZE << pool->order), DMA_BIDIRECTIONAL);

		kbdev->mgm_dev->ops.mgm_free_page(kbdev->mgm_dev, pool->group_id, p, pool->order);

		pool_dbg(pool, "freed page to kernel\n");
	}
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_free_page);

static size_t kbase_mem_pool_shrink_locked(struct kbase_mem_pool *pool, size_t nr_to_shrink)
{
	struct page *p;
	size_t i;

	lockdep_assert_held(&pool->pool_lock);

	for (i = 0; i < nr_to_shrink && !kbase_mem_pool_is_empty(pool); i++) {
		p = kbase_mem_pool_remove_locked(pool, FREE_IN_PROGRESS);
		kbase_mem_pool_free_page(pool, p);
	}

	/* Freeing of pages will be deferred when page migration is enabled. */
	enqueue_free_pool_pages_work(pool);

	return i;
}

static size_t kbase_mem_pool_shrink(struct kbase_mem_pool *pool, size_t nr_to_shrink)
{
	size_t nr_freed;

	kbase_mem_pool_lock(pool);
	if (!pool->dying)
		kbase_mem_pool_free_pages_from_defer_list_locked(pool, false);

	nr_freed = kbase_mem_pool_shrink_locked(pool, nr_to_shrink);
	kbase_mem_pool_unlock(pool);

	return nr_freed;
}

int kbase_mem_pool_grow(struct kbase_mem_pool *pool, size_t nr_to_grow,
			struct task_struct *page_owner)
{
	struct page *p;
	size_t i;

	kbase_mem_pool_lock(pool);

	pool->reclaim_allowed = false;

	if (!pool->dying)
		kbase_mem_pool_free_pages_from_defer_list_locked(pool, false);

	for (i = 0; i < nr_to_grow; i++) {
		if (pool->dying) {
			if (pool->pool_supports_reclaim)
				pool->reclaim_allowed = true;

			kbase_mem_pool_shrink_locked(pool, nr_to_grow);
			kbase_mem_pool_unlock(pool);
			if (page_owner)
				dev_info(pool->kbdev->dev, "%s : Ctx of process %s/%d dying",
					 __func__, page_owner->comm, task_pid_nr(page_owner));

			return -EPERM;
		}
		kbase_mem_pool_unlock(pool);

		if (unlikely(!can_alloc_page(pool, page_owner)))
			return -EPERM;

		p = kbase_mem_alloc_page(pool);
		if (!p) {
			if (pool->pool_supports_reclaim) {
				kbase_mem_pool_lock(pool);
				pool->reclaim_allowed = true;
				kbase_mem_pool_unlock(pool);
			}

			return -ENOMEM;
		}

		kbase_mem_pool_lock(pool);
		kbase_mem_pool_add_locked(pool, p);
	}

	if (pool->pool_supports_reclaim)
		pool->reclaim_allowed = true;

	kbase_mem_pool_unlock(pool);

	return 0;
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_grow);

void kbase_mem_pool_trim(struct kbase_mem_pool *pool, size_t new_size)
{
	size_t cur_size;
	int err = 0;

	cur_size = kbase_mem_pool_size(pool);

	if (new_size > pool->max_size)
		new_size = pool->max_size;

	if (new_size < cur_size)
		kbase_mem_pool_shrink(pool, cur_size - new_size);
	else if (new_size > cur_size)
		err = kbase_mem_pool_grow(pool, new_size - cur_size, NULL);

	if (err) {
		size_t grown_size = kbase_mem_pool_size(pool);

		dev_warn(
			pool->kbdev->dev,
			"Mem pool not grown to the required size of %zu bytes, grown for additional %zu bytes instead!\n",
			(new_size - cur_size), (grown_size - cur_size));
	}
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_trim);

void kbase_mem_pool_set_max_size(struct kbase_mem_pool *pool, size_t max_size)
{
	size_t cur_size;
	size_t nr_to_shrink;

	kbase_mem_pool_lock(pool);

	pool->max_size = max_size;

	cur_size = kbase_mem_pool_size(pool);
	if (max_size < cur_size) {
		nr_to_shrink = cur_size - max_size;
		kbase_mem_pool_shrink_locked(pool, nr_to_shrink);
	}

	kbase_mem_pool_unlock(pool);
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_set_max_size);

static unsigned long kbase_mem_pool_reclaim_count_objects(struct shrinker *s,
							  struct shrink_control *sc)
{
	struct kbase_mem_pool *pool;
	size_t pool_size;

	CSTD_UNUSED(sc);

	pool = KBASE_GET_KBASE_DATA_FROM_SHRINKER(s, struct kbase_mem_pool, reclaim);

	/* Pools not supporting reclaims are not assumed to register reclaim callbacks */
	if (WARN_ON(!pool->pool_supports_reclaim))
		return 0;

	kbase_mem_pool_lock(pool);
	if (!pool->reclaim_allowed && !pool->dying) {
		kbase_mem_pool_unlock(pool);
		/* Tell shrinker to skip reclaim
		 * even though freeable pages are available
		 */
		return 0;
	}
	pool_size = kbase_mem_pool_size(pool);
	kbase_mem_pool_unlock(pool);

	return pool_size;
}

static unsigned long kbase_mem_pool_reclaim_scan_objects(struct shrinker *s,
							 struct shrink_control *sc)
{
	struct kbase_mem_pool *pool;
	unsigned long freed;

	pool = KBASE_GET_KBASE_DATA_FROM_SHRINKER(s, struct kbase_mem_pool, reclaim);

	if (WARN_ON(!pool->pool_supports_reclaim))
		return SHRINK_STOP;

	kbase_mem_pool_lock(pool);
	if (!pool->reclaim_allowed && !pool->dying) {
		kbase_mem_pool_unlock(pool);
		/* Tell shrinker that reclaim can't be done, and
		 * do not attempt again for this reclaim context.
		 */
		return SHRINK_STOP;
	}

	pool_dbg(pool, "reclaim scan %ld:\n", sc->nr_to_scan);

	freed = kbase_mem_pool_shrink_locked(pool, sc->nr_to_scan);

	kbase_mem_pool_unlock(pool);

	pool_dbg(pool, "reclaim freed %ld pages\n", freed);

	return freed;
}

static int kbasep_mem_pool_init(struct kbase_mem_pool *pool, size_t max_size, unsigned int order,
				int group_id, struct kbase_device *kbdev, bool support_reclaim)
{
	struct shrinker *reclaim;

	if (WARN_ON(group_id < 0) || WARN_ON(group_id >= MEMORY_GROUP_MANAGER_NR_GROUPS)) {
		return -EINVAL;
	}

	pool->cur_size = 0;
	INIT_LIST_HEAD(&pool->link_to_ctrl);
	pool->max_size = max_size;
	atomic_set(&pool->deferred_size, 0);
	pool->order = order;
	pool->group_id = group_id;
	pool->kbdev = kbdev;
	pool->dying = false;
	pool->pool_supports_reclaim = support_reclaim;
	pool->reclaim_allowed = false;
	atomic_set(&pool->isolation_in_progress_cnt, 0);
	atomic_set(&pool->defer_seq, 0);

	spin_lock_init(&pool->pool_lock);
	INIT_LIST_HEAD(&pool->page_list);
	INIT_LIST_HEAD(&pool->deferred_pages_list);

	if (support_reclaim) {
		reclaim = KBASE_INIT_RECLAIM(pool, reclaim, "mali-mem-pool");
		if (!reclaim)
			return -ENOMEM;
		KBASE_SET_RECLAIM(pool, reclaim, reclaim);

		reclaim->count_objects = kbase_mem_pool_reclaim_count_objects;
		reclaim->scan_objects = kbase_mem_pool_reclaim_scan_objects;
		reclaim->seeks = DEFAULT_SEEKS;
		reclaim->batch = 0;

		KBASE_REGISTER_SHRINKER(reclaim, "mali-mem-pool", pool);
	}

	pool_dbg(pool, "initialized\n");

	return 0;
}

int kbase_mem_pool_init(struct kbase_mem_pool *pool, size_t max_size, unsigned int order,
			int group_id, struct kbase_device *kbdev)
{
	return kbasep_mem_pool_init(pool, max_size, order, group_id, kbdev, true);
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_init);

int kbase_mem_pool_init_no_reclaim(struct kbase_mem_pool *pool, size_t max_size, unsigned int order,
				   int group_id, struct kbase_device *kbdev)
{
	return kbasep_mem_pool_init(pool, max_size, order, group_id, kbdev, false);
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_init_no_reclaim);

void kbase_mem_pool_mark_dying(struct kbase_mem_pool *pool)
{
	kbase_mem_pool_lock(pool);
	pool->dying = true;
	/* Remove the pool from pmode pages defer control */
	kbase_csf_scheduler_pages_defer_ctrl_drop_pool(pool, false);
	kbase_mem_pool_unlock(pool);
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_mark_dying);

void kbase_mem_pool_term(struct kbase_mem_pool *pool)
{
	struct page *p, *tmp;
	struct kbase_device *kbdev = pool->kbdev;
	struct kbase_csf_protm_mem_pages_defer_ctrl *pages_defer_ctrl =
		&kbdev->csf.scheduler.pages_defer_ctrl;
	unsigned int time_out_ms = kbase_get_timeout_ms(kbdev, CSF_SCHED_PROTM_PROGRESS_TIMEOUT) +
				   kbase_get_timeout_ms(kbdev, CSF_GPU_RESET_TIMEOUT);
	long remaining = (long)msecs_to_jiffies(time_out_ms);
	LIST_HEAD(spill_list);
	LIST_HEAD(free_list);

	pool_dbg(pool, "terminate()\n");

	if (pool->pool_supports_reclaim)
		KBASE_UNREGISTER_SHRINKER(pool->reclaim);

	/* By taking the pool lock, the ownership is established for pool related ops. */
	kbase_mem_pool_lock(pool);
	pool->max_size = 0;

	/* Remove it from defer control */
	kbase_csf_scheduler_pages_defer_ctrl_drop_pool(pool, false);

	/* if the pool has deferred pages, has to wait for the pmode to complete or a reset */
	while (atomic_read(&pool->deferred_size) && remaining) {
		kbase_mem_pool_unlock(pool);

		remaining = wait_event_timeout(pages_defer_ctrl->pools_term_wq,
					       is_csf_scheduler_protm_seq_completed(
						       pool->kbdev, atomic_read(&pool->defer_seq)),
					       remaining);

		kbase_mem_pool_lock(pool);
		if (is_csf_scheduler_protm_seq_completed(pool->kbdev,
							 atomic_read(&pool->defer_seq)))
			break;
	}

	if (atomic_read(&pool->deferred_size) &&
	    !is_csf_scheduler_protm_seq_completed(pool->kbdev, atomic_read(&pool->defer_seq))) {
		/* This should not happen as the wait time is assumed able to ensure at least a
		 * pmode-quit or a reset. Proceed to force releasing of the pages as a last resort
		 * recovery for the unexpected condition. This is achieved by the pool having
		 * already been removed from defer_ctrl list earlier on.
		 */
		dev_err(kbdev->dev,
			"%s timeout on waiting for defer_seq(%d) to complete: curr_seq=%d",
			__func__, atomic_read(&pool->defer_seq),
			kbase_csf_scheduler_get_protm_seq_num(kbdev));
	}

	/* Proceed to release the deferred pages */
	kbase_mem_pool_free_pages_from_defer_list_locked(pool, false);

	/* Free normal pool pages */
	while (!kbase_mem_pool_is_empty(pool)) {
		/* Free remaining pages to kernel */
		p = kbase_mem_pool_remove_locked(pool, FREE_IN_PROGRESS);
		if (p)
			list_add(&p->lru, &free_list);
	}

	kbase_mem_pool_unlock(pool);

	list_for_each_entry_safe(p, tmp, &free_list, lru) {
		list_del_init(&p->lru);
		kbase_mem_pool_free_page(pool, p);
	}

	/* Freeing of pages will be deferred when page migration is enabled. */
	enqueue_free_pool_pages_work(pool);

	/* Before returning wait to make sure there are no pages undergoing page isolation
	 * which will require reference to this pool.
	 */
	if (kbase_is_page_migration_enabled()) {
		while (atomic_read(&pool->isolation_in_progress_cnt))
			cpu_relax();
	}
	pool_dbg(pool, "terminated\n");
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_term);

struct page *kbase_mem_pool_alloc(struct kbase_mem_pool *pool)
{
	struct page *p;

	pool_dbg(pool, "alloc()\n");
	p = kbase_mem_pool_remove(pool, ALLOCATE_IN_PROGRESS);

	return p;
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_alloc);

struct page *kbase_mem_pool_alloc_locked(struct kbase_mem_pool *pool)
{
	lockdep_assert_held(&pool->pool_lock);

	pool_dbg(pool, "alloc_locked()\n");
	return kbase_mem_pool_remove_locked(pool, ALLOCATE_IN_PROGRESS);
}

void kbase_mem_pool_free(struct kbase_mem_pool *pool, struct page *p, bool dirty)
{
	pool_dbg(pool, "free()\n");

	if (kbase_mem_pool_add_deferred_if_required(pool, p))
		return;

	if (!kbase_mem_pool_is_full(pool)) {
		/* Add to our own pool */
		if (dirty)
			kbase_mem_pool_sync_page(pool, p);

		kbase_mem_pool_add(pool, p);
	} else {
		/* Free page */
		kbase_mem_pool_free_page(pool, p);
		/* Freeing of pages will be deferred when page migration is enabled. */
		enqueue_free_pool_pages_work(pool);
	}
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_free);

void kbase_mem_pool_free_locked(struct kbase_mem_pool *pool, struct page *p, bool dirty)
{
	pool_dbg(pool, "free_locked()\n");

	lockdep_assert_held(&pool->pool_lock);

	if (kbase_mem_pool_add_deferred_if_required_locked(pool, p))
		return;

	if (!kbase_mem_pool_is_full(pool)) {
		/* Add to our own pool */
		if (dirty)
			kbase_mem_pool_sync_page(pool, p);

		kbase_mem_pool_add_locked(pool, p);
	} else {
		/* Free page */
		kbase_mem_pool_free_page(pool, p);
		/* Freeing of pages will be deferred when page migration is enabled. */
		enqueue_free_pool_pages_work(pool);
	}
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_free_locked);

int kbase_mem_pool_alloc_pages(struct kbase_mem_pool *pool, size_t nr_small_pages,
			       struct tagged_addr *pages, bool partial_allowed,
			       struct task_struct *page_owner)
{
	struct page *p;
	size_t nr_from_pool;
	size_t i = 0;
	int err = -ENOMEM;
	size_t nr_pages_internal;

	nr_pages_internal = nr_small_pages / (1u << (pool->order));

	if (nr_pages_internal * (1u << pool->order) != nr_small_pages)
		return -EINVAL;

	pool_dbg(pool, "alloc_pages(small=%zu):\n", nr_small_pages);
	pool_dbg(pool, "alloc_pages(internal=%zu):\n", nr_pages_internal);

	/* Get pages from this pool */
	kbase_mem_pool_lock(pool);
	nr_from_pool = min(nr_pages_internal, kbase_mem_pool_size(pool));

	while (nr_from_pool--) {
		uint j;

		p = kbase_mem_pool_remove_locked(pool, ALLOCATE_IN_PROGRESS);

		if (pool->order) {
			pages[i++] = as_tagged_tag(page_to_phys(p), HUGE_HEAD | HUGE_PAGE);
			for (j = 1; j < (1u << pool->order); j++)
				pages[i++] =
					as_tagged_tag(page_to_phys(p) + PAGE_SIZE * j, HUGE_PAGE);
		} else {
			pages[i++] = as_tagged(page_to_phys(p));
		}
	}
	kbase_mem_pool_unlock(pool);

	/* Get any remaining pages from kernel */
	while (i != nr_small_pages) {
		if (unlikely(!can_alloc_page(pool, page_owner)))
			goto err_rollback;

		p = kbase_mem_alloc_page(pool);
		if (!p) {
			if (partial_allowed)
				goto done;
			else
				goto err_rollback;
		}

		if (pool->order) {
			uint j;

			pages[i++] = as_tagged_tag(page_to_phys(p), HUGE_PAGE | HUGE_HEAD);
			for (j = 1; j < (1u << pool->order); j++) {
				phys_addr_t phys;

				phys = page_to_phys(p) + PAGE_SIZE * j;
				pages[i++] = as_tagged_tag(phys, HUGE_PAGE);
			}
		} else {
			pages[i++] = as_tagged(page_to_phys(p));
		}
	}

done:
	pool_dbg(pool, "alloc_pages(%zu) done\n", i);
	return i;

err_rollback:
	kbase_mem_pool_free_pages(pool, i, pages, NOT_DIRTY, NOT_RECLAIMED);
	return err;
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_alloc_pages);

int kbase_mem_pool_alloc_pages_locked(struct kbase_mem_pool *pool, size_t nr_small_pages,
				      struct tagged_addr *pages)
{
	struct page *p;
	size_t i;
	size_t nr_pages_internal;

	lockdep_assert_held(&pool->pool_lock);

	nr_pages_internal = nr_small_pages / (1u << (pool->order));

	if (nr_pages_internal * (1u << pool->order) != nr_small_pages)
		return -EINVAL;

	pool_dbg(pool, "alloc_pages_locked(small=%zu):\n", nr_small_pages);
	pool_dbg(pool, "alloc_pages_locked(internal=%zu):\n", nr_pages_internal);

	if (kbase_mem_pool_size(pool) < nr_pages_internal) {
		pool_dbg(pool, "Failed alloc\n");
		return -ENOMEM;
	}

	for (i = 0; i < nr_pages_internal; i++) {
		uint j;

		p = kbase_mem_pool_remove_locked(pool, ALLOCATE_IN_PROGRESS);
		if (pool->order) {
			*pages++ = as_tagged_tag(page_to_phys(p), HUGE_HEAD | HUGE_PAGE);
			for (j = 1; j < (1u << pool->order); j++) {
				*pages++ =
					as_tagged_tag(page_to_phys(p) + PAGE_SIZE * j, HUGE_PAGE);
			}
		} else {
			*pages++ = as_tagged(page_to_phys(p));
		}
	}

	return nr_small_pages;
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_alloc_pages_locked);

static void kbase_mem_pool_add_array(struct kbase_mem_pool *pool, size_t nr_pages,
				     struct tagged_addr *pages, bool zero, bool sync)
{
	struct page *p;
	size_t nr_to_pool = 0;
	LIST_HEAD(new_page_list);
	size_t i;

	if (!nr_pages)
		return;

	pool_dbg(pool, "add_array(%zu, zero=%d, sync=%d):\n", nr_pages, zero, sync);

	/* Zero/sync pages first without holding the pool lock */
	for (i = 0; i < nr_pages; i++) {
		if (unlikely(!is_valid_addr(pages[i])))
			continue;

		if (is_huge_head(pages[i]) || !is_huge(pages[i])) {
			p = as_page(pages[i]);
			if (zero)
				kbase_mem_pool_zero_page(pool, p);
			else if (sync)
				kbase_mem_pool_sync_page(pool, p);

			list_add(&p->lru, &new_page_list);
			nr_to_pool++;
		}
		pages[i] = as_tagged(KBASE_INVALID_PHYSICAL_ADDRESS);
	}

	/* Add new page list to pool */
	kbase_mem_pool_add_list(pool, &new_page_list, nr_to_pool);

	pool_dbg(pool, "add_array(%zu) added %zu pages\n", nr_pages, nr_to_pool);
}

static void kbase_mem_pool_add_array_locked(struct kbase_mem_pool *pool, size_t nr_pages,
					    struct tagged_addr *pages, bool zero, bool sync)
{
	struct page *p;
	size_t nr_to_pool = 0;
	LIST_HEAD(new_page_list);
	size_t i;

	lockdep_assert_held(&pool->pool_lock);

	if (!nr_pages)
		return;

	pool_dbg(pool, "add_array_locked(%zu, zero=%d, sync=%d):\n", nr_pages, zero, sync);

	/* Zero/sync pages first */
	for (i = 0; i < nr_pages; i++) {
		if (unlikely(!is_valid_addr(pages[i])))
			continue;

		if (is_huge_head(pages[i]) || !is_huge(pages[i])) {
			p = as_page(pages[i]);
			if (zero)
				kbase_mem_pool_zero_page(pool, p);
			else if (sync)
				kbase_mem_pool_sync_page(pool, p);

			list_add(&p->lru, &new_page_list);
			nr_to_pool++;
		}
		pages[i] = as_tagged(KBASE_INVALID_PHYSICAL_ADDRESS);
	}

	/* Add new page list to pool */
	kbase_mem_pool_add_list_locked(pool, &new_page_list, nr_to_pool);

	pool_dbg(pool, "add_array_locked(%zu) added %zu pages\n", nr_pages, nr_to_pool);
}

void kbase_mem_pool_free_pages(struct kbase_mem_pool *pool, size_t nr_pages,
			       struct tagged_addr *pages, bool dirty, bool reclaimed)
{
	struct page *p;
	size_t nr_to_pool;
	LIST_HEAD(to_pool_list);
	size_t i = 0;
	bool pages_released = false;

	pool_dbg(pool, "free_pages(%zu):\n", nr_pages);
	if (kbase_mem_is_pmode_deferral_required(pool->kbdev)) {
		kbase_mem_pool_add_array_deferred(pool, nr_pages, pages, false, dirty);
		pool_dbg(pool, "free_pages(%zu) done\n", nr_pages);
		return;
	}

	if (!reclaimed) {
		/* Add to this pool */
		nr_to_pool = kbase_mem_pool_capacity(pool) << pool->order;
		nr_to_pool = min(nr_pages, nr_to_pool);

		kbase_mem_pool_add_array(pool, nr_to_pool, pages, false, dirty);

		i += nr_to_pool;
	}

	/* Free any remaining pages to kernel */
	for (; i < nr_pages; i++) {
		if (unlikely(!is_valid_addr(pages[i])))
			continue;

		if (is_huge(pages[i]) && !is_huge_head(pages[i])) {
			pages[i] = as_tagged(KBASE_INVALID_PHYSICAL_ADDRESS);
			continue;
		}
		p = as_page(pages[i]);

		kbase_mem_pool_free_page(pool, p);
		pages[i] = as_tagged(KBASE_INVALID_PHYSICAL_ADDRESS);
		pages_released = true;
	}

	/* Freeing of pages will be deferred when page migration is enabled. */
	if (pages_released)
		enqueue_free_pool_pages_work(pool);

	pool_dbg(pool, "free_pages(%zu) done\n", nr_pages);
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_free_pages);

void kbase_mem_pool_free_pages_locked(struct kbase_mem_pool *pool, size_t nr_pages,
				      struct tagged_addr *pages, bool dirty, bool reclaimed)
{
	struct page *p;
	size_t nr_to_pool;
	LIST_HEAD(to_pool_list);
	size_t i = 0;
	bool pages_released = false;

	lockdep_assert_held(&pool->pool_lock);

	pool_dbg(pool, "free_pages_locked(%zu):\n", nr_pages);
	if (kbase_mem_is_pmode_deferral_required(pool->kbdev)) {
		kbase_mem_pool_add_array_deferred_locked(pool, nr_pages, pages, false, dirty);
		goto done;
	}

	if (!reclaimed) {
		/* Add to this pool */
		nr_to_pool = kbase_mem_pool_capacity(pool) << pool->order;
		nr_to_pool = min(nr_pages, nr_to_pool);

		kbase_mem_pool_add_array_locked(pool, nr_to_pool, pages, false, dirty);

		i += nr_to_pool;
	}

	/* Free any remaining pages to kernel */
	for (; i < nr_pages; i++) {
		if (unlikely(!is_valid_addr(pages[i])))
			continue;

		if (is_huge(pages[i]) && !is_huge_head(pages[i])) {
			pages[i] = as_tagged(KBASE_INVALID_PHYSICAL_ADDRESS);
			continue;
		}

		p = as_page(pages[i]);

		kbase_mem_pool_free_page(pool, p);
		pages[i] = as_tagged(KBASE_INVALID_PHYSICAL_ADDRESS);
		pages_released = true;
	}

	/* Freeing of pages will be deferred when page migration is enabled. */
	if (pages_released)
		enqueue_free_pool_pages_work(pool);
done:
	pool_dbg(pool, "free_pages_locked(%zu) done\n", nr_pages);
}
KBASE_EXPORT_TEST_API(kbase_mem_pool_free_pages_locked);
