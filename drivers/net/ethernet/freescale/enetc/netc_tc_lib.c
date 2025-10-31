// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NETC NTMP (NETC Table Management Protocol) 2.0 driver
 * Copyright 2024 NXP
 */
#include <linux/fsl/netc_lib.h>

#include "ntmp_private.h"

#define SDU_TYPE_MPDU				1

struct netc_flower_rule *
netc_find_flower_rule_by_cookie(struct ntmp_priv *priv, int port_id,
				unsigned long cookie)
{
	struct netc_flower_rule *rule;

	hlist_for_each_entry(rule, &priv->flower_list, node) {
		if (priv->dev_type == NETC_DEV_SWITCH) {
			if (rule->port_id == port_id && rule->cookie == cookie)
				return rule;
		} else {
			if (rule->cookie == cookie)
				return rule;
		}
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(netc_find_flower_rule_by_cookie);

static bool netc_flower_isit_key_matched(const struct isit_keye_data *key1,
					 const struct isit_keye_data *key2)
{
	if (memcmp(key1, key2, sizeof(*key1)))
		return false;

	return true;
}

static bool netc_flower_ipft_key_matched(const struct ipft_keye_data *key1,
					 const struct ipft_keye_data *key2)
{
	const void *key1_start = &key1->frm_attr_flags;
	const void *key2_start = &key2->frm_attr_flags;
	u32 size = sizeof(*key1) - 8;

	if (memcmp(key1_start, key2_start, size))
		return false;

	return true;
}

struct netc_flower_rule *
netc_find_flower_rule_by_key(struct ntmp_priv *priv,
			     enum netc_key_tbl_type tbl_type,
			     void *key)
{
	struct netc_flower_key_tbl *key_tbl;
	struct netc_flower_rule *rule;

	hlist_for_each_entry(rule, &priv->flower_list, node) {
		key_tbl = rule->key_tbl;
		if (key_tbl->tbl_type != tbl_type)
			continue;

		if (tbl_type == FLOWER_KEY_TBL_ISIT &&
		    netc_flower_isit_key_matched(key, &key_tbl->isit_entry->keye))
			return rule;
		else if (tbl_type == FLOWER_KEY_TBL_IPFT &&
			 netc_flower_ipft_key_matched(key, &key_tbl->ipft_entry->keye))
			return rule;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(netc_find_flower_rule_by_key);

static int netc_psfp_flower_key_validate(struct ntmp_priv *priv,
					 struct isit_keye_data *keye, int prio,
					 struct netc_flower_key_tbl **key_tbl,
					 struct netlink_ext_ack *extack)
{
	struct netc_flower_rule *rule, *tmp_rule;
	struct netc_flower_key_tbl *tmp_tbl;

	/* Find the first rule with the same ISIT key */
	rule = netc_find_flower_rule_by_key(priv, FLOWER_KEY_TBL_ISIT, keye);
	if (!rule)
		return 0;

	if (rule->flower_type != FLOWER_TYPE_PSFP) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Cannot add new rule with different flower type");
		return -EINVAL;
	}

	if (prio < 0) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Rule conflicts with existing rules");
		return -EINVAL;
	}

	/* Unsupport if existing rule does not have ISFT entry */
	if (!rule->isft_entry) {
		NL_SET_ERR_MSG_MOD(extack,
				   "VLAN pbit in rule conflicts with existing rule");
		return -EINVAL;
	}

	/* If there are other rules using the same key, an error is returned */
	hlist_for_each_entry(tmp_rule, &priv->flower_list, node) {
		tmp_tbl = tmp_rule->key_tbl;
		if (tmp_tbl->tbl_type != FLOWER_KEY_TBL_ISIT)
			continue;

		if (!netc_flower_isit_key_matched(keye, &tmp_tbl->isit_entry->keye))
			continue;

		if (tmp_rule->isft_entry &&
		    FIELD_GET(ISFT_PCP, tmp_rule->isft_entry->keye.pcp) == prio) {
			NL_SET_ERR_MSG_MOD(extack,
					   "The same key has been used by existing rule");
			return -EINVAL;
		}
	}

	*key_tbl = rule->key_tbl;

	return 0;
}

static struct netc_gate_tbl *
netc_find_flower_gate_table(struct ntmp_priv *priv, u32 index)
{
	struct netc_flower_rule *rule;

	hlist_for_each_entry(rule, &priv->flower_list, node) {
		struct netc_gate_tbl *gate_tbl = rule->gate_tbl;

		if (!gate_tbl)
			continue;

		if (gate_tbl->sgit_entry->entry_id == index)
			return gate_tbl;
	}

	return NULL;
}

static int netc_psfp_gate_entry_validate(struct ntmp_priv *priv,
					 struct flow_action_entry *gate_entry,
					 struct netc_gate_tbl **gate_tbl,
					 struct netlink_ext_ack *extack)
{
	u64 max_cycle_time;
	u32 num_gates;

	if (!gate_entry) {
		NL_SET_ERR_MSG_MOD(extack, "No gate entries");
		return -EINVAL;
	}

	num_gates = gate_entry->gate.num_entries;
	if (num_gates > SGCLT_MAX_GE_NUM) {
		NL_SET_ERR_MSG_MOD(extack, "Gate number exceeds 256");
		return -EINVAL;
	}

	max_cycle_time = gate_entry->gate.cycletime + gate_entry->gate.cycletimeext;
	if (max_cycle_time > SGIT_MAX_CT_PLUS_CT_EXT) {
		NL_SET_ERR_MSG_MOD(extack, "Max cycle time exceeds 0x3ffffff ns");
		return -EINVAL;
	}

	if (gate_entry->hw_index >= priv->caps.sgit_num_entries) {
		NL_SET_ERR_MSG_FMT_MOD(extack, "Gate hw index cannot exceed %u",
				       priv->caps.sgit_num_entries - 1);
		return -EINVAL;
	}

	if (test_and_set_bit(gate_entry->hw_index, priv->sgit_eid_bitmap))
		*gate_tbl = netc_find_flower_gate_table(priv,
							gate_entry->hw_index);

	return 0;
}

static struct netc_police_tbl *
netc_find_flower_police_table(struct ntmp_priv *priv, u32 index)
{
	struct netc_flower_rule *rule;

	hlist_for_each_entry(rule, &priv->flower_list, node) {
		struct netc_police_tbl *police_tbl = rule->police_tbl;

		if (!police_tbl)
			continue;

		if (police_tbl->rpt_entry->entry_id == index)
			return police_tbl;
	}

	return NULL;
}

int netc_police_entry_validate(struct ntmp_priv *priv,
			       const struct flow_action *action,
			       const struct flow_action_entry *police_entry,
			       struct netc_police_tbl **police_tbl,
			       struct netlink_ext_ack *extack)
{
	if (police_entry->police.exceed.act_id != FLOW_ACTION_DROP) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when exceed action is not drop");
		return -EOPNOTSUPP;
	}

	if (police_entry->police.notexceed.act_id != FLOW_ACTION_PIPE &&
	    police_entry->police.notexceed.act_id != FLOW_ACTION_ACCEPT) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when conform action is not pipe or ok");
		return -EOPNOTSUPP;
	}

	if (police_entry->police.notexceed.act_id == FLOW_ACTION_ACCEPT &&
	    !flow_action_is_last_entry(action, police_entry)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when conform action is ok, but action is not last");
		return -EOPNOTSUPP;
	}

	if (police_entry->police.peakrate_bytes_ps ||
	    police_entry->police.avrate || police_entry->police.overhead) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when peakrate/avrate/overhead is configured");
		return -EOPNOTSUPP;
	}

	if (police_entry->police.rate_pkt_ps) {
		NL_SET_ERR_MSG_MOD(extack,
				   "QoS offload not support packets per second");
		return -EOPNOTSUPP;
	}

	if (!police_entry->police.rate_bytes_ps && !police_entry->police.burst) {
		NL_SET_ERR_MSG_MOD(extack, "Burst and rate cannot be all 0");
		return -EINVAL;
	}

	if (police_entry->hw_index >= priv->caps.rpt_num_entries) {
		NL_SET_ERR_MSG_FMT_MOD(extack, "Police index cannot exceed %u",
				       priv->caps.rpt_num_entries - 1);
		return -EINVAL;
	}

	if (test_and_set_bit(police_entry->hw_index, priv->rpt_eid_bitmap))
		*police_tbl = netc_find_flower_police_table(priv,
							    police_entry->hw_index);

	return 0;
}
EXPORT_SYMBOL_GPL(netc_police_entry_validate);

static int netc_psfp_isit_keye_construct(struct flow_rule *rule, int port_index,
					 struct isit_keye_data *keye, int *prio,
					 struct netlink_ext_ack *extack)
{
	struct flow_match_eth_addrs addr_match = {0};
	struct flow_match_vlan vlan_match = {0};
	struct isit_psfp_frame_key *frame_key;
	u32 key_aux;
	u16 vlan;

	frame_key = (struct isit_psfp_frame_key *)keye->frame_key;
	/* For ENETC, the port_index should be 0 */
	key_aux = FIELD_PREP(ISIT_SRC_PORT_ID, port_index);

	if (!flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_ETH_ADDRS)) {
		NL_SET_ERR_MSG_MOD(extack, "Unsupported, must include ETH_ADDRS");
		return -EINVAL;
	}

	flow_rule_match_eth_addrs(rule, &addr_match);
	if (!is_zero_ether_addr(addr_match.mask->dst) &&
	    !is_zero_ether_addr(addr_match.mask->src)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Cannot match on both source and destination MAC");
		return -EINVAL;
	}

	if (!is_zero_ether_addr(addr_match.mask->dst)) {
		if (!is_broadcast_ether_addr(addr_match.mask->dst)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Masked matching on destination MAC not supported");
			return -EINVAL;
		}

		ether_addr_copy(frame_key->mac, addr_match.key->dst);
		key_aux |= FIELD_PREP(ISIT_KEY_TYPE, ISIT_KEY_TYPE1_DMAC_VLAN);
	}

	if (!is_zero_ether_addr(addr_match.mask->src)) {
		if (!is_broadcast_ether_addr(addr_match.mask->src)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Masked matching on source MAC not supported");
			return -EINVAL;
		}

		ether_addr_copy(frame_key->mac, addr_match.key->src);
		key_aux |= FIELD_PREP(ISIT_KEY_TYPE, ISIT_KEY_TYPE0_SMAC_VLAN);
	}

	keye->key_aux = cpu_to_le32(key_aux);

	if (!flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_VLAN))
		return 0;

	flow_rule_match_vlan(rule, &vlan_match);
	if (vlan_match.mask->vlan_id) {
		if (vlan_match.mask->vlan_id != VLAN_VID_MASK) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Only full mask is supported for VLAN ID");
			return -EINVAL;
		}

		vlan = vlan_match.key->vlan_id;
		vlan |= BIT(15);
		frame_key->vlan_h = (vlan >> 8) & 0xff;
		frame_key->vlan_l = vlan & 0xff;
	}

	if (vlan_match.mask->vlan_priority) {
		if (vlan_match.mask->vlan_priority !=
		    (VLAN_PRIO_MASK >> VLAN_PRIO_SHIFT)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Only full mask is supported for VLAN priority");
			return -EINVAL;
		}

		*prio = vlan_match.key->vlan_priority;
	}

	return 0;
}

static void netc_psfp_gate_entry_config(struct ntmp_priv *priv,
					struct flow_action_entry *gate_entry,
					struct ntmp_sgit_entry *sgit_entry,
					struct ntmp_sgclt_entry *sgclt_entry)
{
	u32 cycle_time_ext = gate_entry->gate.cycletimeext;
	u32 num_gates = gate_entry->gate.num_entries;
	u32 cycle_time = gate_entry->gate.cycletime;
	u64 base_time = gate_entry->gate.basetime;
	u8 sgit_cfg, sgit_icfg = SGIT_GST;
	u8 sgclt_extcfg = SGCLT_EXT_GTST;
	int i;

	if (gate_entry->gate.prio >= 0) {
		sgit_icfg |= FIELD_PREP(SGIT_IPV, gate_entry->gate.prio);
		sgit_icfg |= SGIT_OIPV;
	}

	if (priv->adjust_base_time)
		base_time = priv->adjust_base_time(priv, base_time, cycle_time);

	sgit_cfg = FIELD_PREP(SGIT_SDU_TYPE, SDU_TYPE_MPDU);
	sgit_entry->acfge.admin_base_time = cpu_to_le64(base_time);
	sgit_entry->acfge.admin_sgcl_eid = cpu_to_le32(sgclt_entry->entry_id);
	sgit_entry->acfge.admin_cycle_time_ext = cpu_to_le32(cycle_time_ext);
	sgit_entry->cfge.cfg = sgit_cfg;
	sgit_entry->icfge.icfg = sgit_icfg;

	sgclt_entry->cfge.cycle_time = cpu_to_le32(cycle_time);
	sgclt_entry->cfge.list_length = num_gates - 1;
	if (gate_entry->gate.prio >= 0) {
		sgclt_extcfg |= FIELD_PREP(SGCLT_EXT_IPV, gate_entry->gate.prio);
		sgclt_extcfg |= SGCLT_EXT_OIPV;
	}
	sgclt_entry->cfge.ext_cfg = sgclt_extcfg;

	for (i = 0; i < num_gates; i++) {
		struct action_gate_entry *from = &gate_entry->gate.entries[i];
		struct sgclt_ge *to = &sgclt_entry->cfge.ge[i];
		u32 sgclt_cfg = 0;

		if (from->gate_state)
			sgclt_cfg |= SGCLT_GTST;

		if (from->ipv >= 0) {
			sgclt_cfg |= FIELD_PREP(SGCLT_IPV, from->ipv);
			sgclt_cfg |= SGCLT_OIPV;
		}

		if (from->maxoctets >= 0) {
			sgclt_cfg |= FIELD_PREP(SGCLT_IOM, from->maxoctets);
			sgclt_cfg |= SGCLT_IOMEN;
		}

		to->interval = cpu_to_le32(from->interval);
		to->cfg = cpu_to_le32(sgclt_cfg);
	}
}

void netc_rpt_entry_config(struct flow_action_entry *police_entry,
			   struct ntmp_rpt_entry *rpt_entry)
{
	u64 rate_bps;
	u32 cir, cbs;
	u16 cfg;

	rpt_entry->entry_id = police_entry->hw_index;

	/* The unit of rate_bytes_ps is 1Bps, the uint of cir is 3.725bps,
	 * so convert it.
	 */
	rate_bps = police_entry->police.rate_bytes_ps * 8;
	cir = div_u64(rate_bps * 1000, 3725);
	cbs = police_entry->police.burst;
	cfg = FIELD_PREP(RPT_SDU_TYPE, SDU_TYPE_MPDU);
	rpt_entry->cfge.cir = cpu_to_le32(cir);
	rpt_entry->cfge.cbs = cpu_to_le32(cbs);
	rpt_entry->cfge.cfg = cpu_to_le16(cfg);
	rpt_entry->fee.fen = RPT_FEN;
}
EXPORT_SYMBOL_GPL(netc_rpt_entry_config);

static int netc_delete_sgclt_entry(struct ntmp_priv *priv, u32 entry_id)
{
	struct ntmp_sgclt_entry *sgclt_entry __free(kfree);
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	u32 max_data_size, max_cfge_size;
	u32 num_gates, entry_size;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return 0;

	max_cfge_size = struct_size_t(struct sgclt_cfge_data, ge,
				      SGCLT_MAX_GE_NUM);
	max_data_size = struct_size(sgclt_entry, cfge.ge, SGCLT_MAX_GE_NUM);
	sgclt_entry = kzalloc(max_data_size, GFP_KERNEL);
	if (!sgclt_entry)
		return -ENOMEM;

	err = ntmp_sgclt_query_entry(cbdrs, entry_id, sgclt_entry, max_cfge_size);
	if (err)
		return err;

	/* entry_size equals to 1 + ROUNDUP(N / 2) where N is number of gates */
	num_gates = sgclt_entry->cfge.list_length + 1;
	entry_size = 1 + DIV_ROUND_UP(num_gates, 2);
	err = ntmp_sgclt_delete_entry(cbdrs, entry_id);
	if (err)
		return err;

	ntmp_clear_words_bitmap(priv->sgclt_word_bitmap, entry_id, entry_size);

	return 0;
}

static int netc_delete_sgit_entry(struct ntmp_priv *priv, u32 entry_id)
{
	struct ntmp_sgit_entry *entry __free(kfree);
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	struct ntmp_sgit_entry new_entry = {0};
	u32 sgcl_eid;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return 0;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	/* Step1: Query the stream gate instance table entry to retrieve
	 * the entry id of the administrative gate control list and the
	 * opertational gate control list.
	 */
	err = ntmp_sgit_query_entry(cbdrs, entry_id, entry);
	if (err)
		return err;

	/* Step2: Update the stream gate instance table entry to set
	 * the entry id of the administrative gate control list to NULL.
	 */
	new_entry.acfge.admin_sgcl_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
	new_entry.entry_id = entry_id;
	err = ntmp_sgit_add_or_update_entry(cbdrs, &new_entry);
	if (err)
		return err;

	/* Step3: Delete the stream gate instance table entry. */
	err = ntmp_sgit_delete_entry(cbdrs, entry_id);
	if (err)
		return err;

	ntmp_clear_eid_bitmap(priv->sgit_eid_bitmap, entry_id);

	/* Step4: Delete the administrative gate control list
	 * and the operational gate control list.
	 */
	sgcl_eid = le32_to_cpu(entry->acfge.admin_sgcl_eid);
	err = netc_delete_sgclt_entry(priv, sgcl_eid);
	if (err)
		return err;

	sgcl_eid = le32_to_cpu(entry->sgise.oper_sgcl_eid);
	err = netc_delete_sgclt_entry(priv, sgcl_eid);

	return err;
}

static int netc_psfp_set_related_tables(struct ntmp_priv *priv,
					struct netc_psfp_tbl_entries *tbl)
{
	struct ntmp_sgclt_entry *sgclt_entry = tbl->sgclt_entry;
	struct ntmp_isit_entry *isit_entry = tbl->isit_entry;
	struct ntmp_isft_entry *isft_entry = tbl->isft_entry;
	struct ntmp_sgit_entry *sgit_entry = tbl->sgit_entry;
	struct ntmp_isct_entry *isct_entry = tbl->isct_entry;
	struct ntmp_ist_entry *ist_entry = tbl->ist_entry;
	struct ntmp_rpt_entry *rpt_entry = tbl->rpt_entry;
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	int err;

	err = ntmp_isct_operate_entry(cbdrs, isct_entry->entry_id,
				      NTMP_CMD_ADD, NULL);
	if (err)
		return err;

	if (sgclt_entry) {
		err = ntmp_sgclt_add_entry(cbdrs, sgclt_entry);
		if (err)
			goto delete_isct_entry;
	}

	if (sgit_entry) {
		err = ntmp_sgit_add_or_update_entry(cbdrs, sgit_entry);
		if (err) {
			if (sgclt_entry)
				ntmp_sgclt_delete_entry(cbdrs,
							sgclt_entry->entry_id);
			goto delete_isct_entry;
		}
	}

	if (rpt_entry) {
		err = ntmp_rpt_add_or_update_entry(cbdrs, rpt_entry);
		if (err)
			goto delete_sgit_entry;
	}

	if (ist_entry) {
		err = ntmp_ist_add_or_update_entry(cbdrs, ist_entry);
		if (err)
			goto delete_rpt_entry;
	}

	if (isft_entry) {
		err = ntmp_isft_add_or_update_entry(cbdrs, true, isft_entry);
		if (err)
			goto delete_ist_entry;
	}

	if (isit_entry) {
		err = ntmp_isit_add_or_update_entry(cbdrs, true, isit_entry);
		if (err)
			goto delete_isft_entry;
	}

	return 0;

delete_isft_entry:
	if (isft_entry)
		ntmp_isft_delete_entry(cbdrs, isft_entry->entry_id);

delete_ist_entry:
	if (ist_entry)
		ntmp_ist_delete_entry(cbdrs, ist_entry->entry_id);

delete_rpt_entry:
	if (rpt_entry)
		ntmp_rpt_delete_entry(cbdrs, rpt_entry->entry_id);

delete_sgit_entry:
	if (sgit_entry)
		netc_delete_sgit_entry(priv, sgit_entry->entry_id);

delete_isct_entry:
	ntmp_isct_operate_entry(cbdrs, isct_entry->entry_id,
				NTMP_CMD_DELETE, NULL);

	return err;
}

void netc_init_ist_entry_eids(struct ntmp_priv *priv,
			      struct ntmp_ist_entry *ist_entry)
{
	struct ist_cfge_data *cfge = &ist_entry->cfge;

	cfge->rp_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
	cfge->sgi_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
	cfge->isc_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);

	if (priv->dev_type == NETC_DEV_SWITCH) {
		cfge->isqg_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
		cfge->ifm_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
		cfge->et_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
	}
}
EXPORT_SYMBOL_GPL(netc_init_ist_entry_eids);

static int netc_add_psfp_key_tbl(struct ntmp_priv *priv,
				 struct netc_flower_key_tbl **key_tbl,
				 struct isit_keye_data *isit_key,
				 struct netlink_ext_ack *extack)
{
	struct netc_flower_key_tbl *new_tbl __free(kfree);
	struct ntmp_isit_entry *isit_entry __free(kfree);
	struct ntmp_ist_entry *ist_entry __free(kfree);
	u32 ist_cfg = 0;

	new_tbl = kzalloc(sizeof(*new_tbl), GFP_KERNEL);
	if (!new_tbl)
		return -ENOMEM;

	isit_entry = kzalloc(sizeof(*isit_entry), GFP_KERNEL);
	if (!isit_entry)
		return -ENOMEM;

	ist_entry = kzalloc(sizeof(*ist_entry), GFP_KERNEL);
	if (!ist_entry)
		return -ENOMEM;

	new_tbl->tbl_type = FLOWER_KEY_TBL_ISIT;
	refcount_set(&new_tbl->refcount, 1);

	ist_entry->entry_id = ntmp_lookup_free_eid(priv->ist_eid_bitmap,
						   priv->caps.ist_num_entries);
	if (ist_entry->entry_id == NTMP_NULL_ENTRY_ID) {
		NL_SET_ERR_MSG_MOD(extack, "No available IST entry is found");
		return -ENOSPC;
	}

	switch (priv->cbdrs.tbl.ist_ver) {
	case NTMP_TBL_VER0:
		if (priv->dev_type == NETC_DEV_SWITCH)
			ist_cfg |= FIELD_PREP(IST_V0_FA, IST_SWITCH_FA_BF);
		else
			ist_cfg |= FIELD_PREP(IST_V0_FA, IST_FA_NO_SI_BITMAP);

		ist_cfg |= FIELD_PREP(IST_V0_SDU_TYPE, SDU_TYPE_MPDU);
		break;
	case NTMP_TBL_VER1:
		if (priv->dev_type == NETC_DEV_SWITCH)
			ist_cfg |= FIELD_PREP(IST_V1_FA, IST_SWITCH_FA_BF);
		else
			ist_cfg |= FIELD_PREP(IST_V1_FA, IST_FA_NO_SI_BITMAP);

		ist_cfg |= FIELD_PREP(IST_V1_SDU_TYPE, SDU_TYPE_MPDU);
		break;
	default:
		NL_SET_ERR_MSG_MOD(extack, "Unknown IST version");
		ntmp_clear_eid_bitmap(priv->ist_eid_bitmap,
				      ist_entry->entry_id);

		return -EINVAL;
	}

	ist_entry->cfge.cfg = cpu_to_le32(ist_cfg);
	netc_init_ist_entry_eids(priv, ist_entry);

	isit_entry->is_eid = cpu_to_le32(ist_entry->entry_id);
	isit_entry->keye = *isit_key;

	new_tbl->isit_entry = no_free_ptr(isit_entry);
	new_tbl->ist_entry = no_free_ptr(ist_entry);
	*key_tbl = no_free_ptr(new_tbl);

	return 0;
}

void netc_free_flower_key_tbl(struct ntmp_priv *priv,
			      struct netc_flower_key_tbl *key_tbl)
{
	struct ntmp_ist_entry *ist_entry;

	if (!key_tbl)
		return;

	ist_entry = key_tbl->ist_entry;
	if (ist_entry) {
		ntmp_clear_eid_bitmap(priv->ist_eid_bitmap, ist_entry->entry_id);
		kfree(key_tbl->ist_entry);
	}

	switch (key_tbl->tbl_type) {
	case FLOWER_KEY_TBL_ISIT:
		kfree(key_tbl->isit_entry);
		break;
	case FLOWER_KEY_TBL_IPFT:
		kfree(key_tbl->ipft_entry);
		break;
	}

	kfree(key_tbl);
}
EXPORT_SYMBOL_GPL(netc_free_flower_key_tbl);

int netc_setup_psfp(struct ntmp_priv *priv, int port_id,
		    struct flow_cls_offload *f)
{
	struct flow_action_entry *gate_entry = NULL, *police_entry = NULL;
	struct flow_rule *cls_rule = flow_cls_offload_flow_rule(f);
	struct ntmp_sgclt_entry *sgclt_entry __free(kfree) = NULL;
	struct netc_police_tbl *police_tbl __free(kfree) = NULL;
	struct ntmp_isft_entry *isft_entry __free(kfree) = NULL;
	struct ntmp_sgit_entry *sgit_entry __free(kfree) = NULL;
	struct ntmp_rpt_entry *rpt_entry __free(kfree) = NULL;
	struct netc_gate_tbl *gate_tbl __free(kfree) = NULL;
	struct netc_flower_rule *rule __free(kfree) = NULL;
	struct netc_flower_key_tbl *reused_key_tbl = NULL;
	struct netlink_ext_ack *extack = f->common.extack;
	struct ntmp_isct_entry *isct_entry __free(kfree);
	struct netc_police_tbl *reused_police_tbl = NULL;
	struct netc_gate_tbl *reused_gate_tbl = NULL;
	struct netc_flower_key_tbl *key_tbl = NULL;
	u32 ist_eid, sgclt_eid, isct_eid, sgit_eid;
	u32 sgclt_entry_size = 0, sgclt_data_size;
	struct ntmp_ist_entry *ist_entry = NULL;
	struct flow_action_entry *action_entry;
	struct netc_psfp_tbl_entries psfp_tbl;
	struct isit_keye_data isit_keye = {0};
	struct ntmp_isit_entry *isit_entry;
	unsigned long cookie = f->cookie;
	u32 ist_cfg = 0, num_gates;
	int i, err, priority = -1;
	u16 msdu = 0;

	guard(mutex)(&priv->flower_lock);
	if (netc_find_flower_rule_by_cookie(priv, port_id, cookie)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Cannot add new rule with same cookie");
		return -EINVAL;
	}

	rule = kzalloc(sizeof(*rule), GFP_KERNEL);
	if (!rule)
		return -ENOMEM;

	rule->port_id = port_id;
	rule->cookie = cookie;
	rule->flower_type = FLOWER_TYPE_PSFP;

	/* Find gate action entry and police action entry*/
	flow_action_for_each(i, action_entry, &cls_rule->action)
		if (action_entry->id == FLOW_ACTION_GATE)
			gate_entry = action_entry;
		else if (action_entry->id == FLOW_ACTION_POLICE)
			police_entry = action_entry;

	err = netc_psfp_gate_entry_validate(priv, gate_entry,
					    &reused_gate_tbl, extack);
	if (err)
		return err;

	if (police_entry) {
		msdu = police_entry->police.mtu;
		err = netc_police_entry_validate(priv, &cls_rule->action,
						 police_entry, &reused_police_tbl,
						 extack);
		if (err)
			goto clear_sgit_eid_bit;
	}

	err = netc_psfp_isit_keye_construct(cls_rule, port_id, &isit_keye,
					    &priority, extack);
	if (err)
		goto clear_rpt_eid_bit;

	err = netc_psfp_flower_key_validate(priv, &isit_keye, priority,
					    &reused_key_tbl, extack);
	if (err)
		goto clear_rpt_eid_bit;

	if (!reused_key_tbl) {
		err = netc_add_psfp_key_tbl(priv, &key_tbl, &isit_keye, extack);
		if (err)
			goto clear_rpt_eid_bit;

		isit_entry = key_tbl->isit_entry;
		ist_entry = key_tbl->ist_entry;
		ist_eid = ist_entry->entry_id;
		ist_cfg = le32_to_cpu(ist_entry->cfge.cfg);
	} else {
		ist_eid = reused_key_tbl->ist_entry->entry_id;
	}

	if (!reused_police_tbl && police_entry) {
		police_tbl = kzalloc(sizeof(*police_tbl), GFP_KERNEL);
		if (!police_tbl) {
			err = -ENOMEM;
			goto free_psfp_key_tbl;
		}

		rpt_entry = kzalloc(sizeof(*rpt_entry), GFP_KERNEL);
		if (!rpt_entry) {
			err = -ENOMEM;
			goto free_psfp_key_tbl;
		}

		netc_rpt_entry_config(police_entry, rpt_entry);
	}

	sgit_eid = gate_entry->hw_index;
	if (reused_gate_tbl)
		goto config_isct;

	gate_tbl = kzalloc(sizeof(*gate_tbl), GFP_KERNEL);
	if (!gate_tbl) {
		err = -ENOMEM;
		goto free_psfp_key_tbl;
	}

	sgit_entry = kzalloc(sizeof(*sgit_entry), GFP_KERNEL);
	if (!sgit_entry) {
		err = -ENOMEM;
		goto free_psfp_key_tbl;
	}

	sgit_entry->entry_id = sgit_eid;
	num_gates = gate_entry->gate.num_entries;
	sgclt_entry_size = 1 + DIV_ROUND_UP(num_gates, 2);
	sgclt_eid = ntmp_lookup_free_words(priv->sgclt_word_bitmap,
					   priv->caps.sgclt_num_words,
					   sgclt_entry_size);
	if (sgclt_eid == NTMP_NULL_ENTRY_ID) {
		NL_SET_ERR_MSG_MOD(extack, "No Stream Gate Control List resource");
		err = -ENOSPC;
		goto free_psfp_key_tbl;
	}

	sgclt_data_size = struct_size(sgclt_entry, cfge.ge, num_gates);
	sgclt_entry = kzalloc(sgclt_data_size, GFP_KERNEL);
	if (!sgclt_entry) {
		err = -ENOMEM;
		goto clear_sgclt_eid_words;
	}

	sgclt_entry->entry_id = sgclt_eid;
	netc_psfp_gate_entry_config(priv, gate_entry, sgit_entry, sgclt_entry);

config_isct:
	isct_eid = ntmp_lookup_free_eid(priv->isct_eid_bitmap,
					priv->caps.isct_num_entries);
	if (isct_eid == NTMP_NULL_ENTRY_ID) {
		NL_SET_ERR_MSG_MOD(extack, "No available ISCT entry is found");
		err = -ENOSPC;
		goto clear_sgclt_eid_words;
	}

	isct_entry = kzalloc(sizeof(*isct_entry), GFP_KERNEL);
	if (!isct_entry) {
		err = -ENOMEM;
		goto clear_isct_eid_bit;
	}

	isct_entry->entry_id = isct_eid;

	/* Determine if an ingress stream filter entry is required */
	if (priority >= 0) {
		u16 isft_cfg = FIELD_PREP(ISFT_SDU_TYPE, SDU_TYPE_MPDU);

		isft_entry = kzalloc(sizeof(*isft_entry), GFP_KERNEL);
		if (!isft_entry) {
			err = -ENOMEM;
			goto clear_isct_eid_bit;
		}

		isft_entry->keye.is_eid = cpu_to_le32(ist_eid);
		isft_entry->keye.pcp = FIELD_PREP(ISFT_PCP, priority);
		isft_entry->cfge.msdu = cpu_to_le16(msdu);
		isft_entry->cfge.isc_eid = cpu_to_le32(isct_eid);
		isft_entry->cfge.sgi_eid = cpu_to_le32(sgit_eid);
		isft_cfg |= ISFT_OSGI;

		if (police_entry) {
			isft_cfg |= ISFT_ORP;
			isft_entry->cfge.rp_eid = cpu_to_le32(police_entry->hw_index);
		}

		isft_entry->cfge.cfg = cpu_to_le16(isft_cfg);

		if (ist_entry)
			ist_cfg |= IST_SFE; /* Enable stream filter */
	} else if (ist_entry) {
		ist_cfg |= IST_OSGI;
		ist_entry->cfge.msdu = cpu_to_le16(msdu);
		ist_entry->cfge.isc_eid = cpu_to_le32(isct_eid);
		ist_entry->cfge.sgi_eid = cpu_to_le32(sgit_eid);

		if (police_entry) {
			ist_cfg |= IST_ORP;
			ist_entry->cfge.rp_eid = cpu_to_le32(police_entry->hw_index);
		}
	}

	if (ist_entry)
		ist_entry->cfge.cfg = cpu_to_le32(ist_cfg);

	psfp_tbl.ist_entry = ist_entry;
	psfp_tbl.rpt_entry = rpt_entry;
	psfp_tbl.isit_entry = isit_entry;
	psfp_tbl.isft_entry = isft_entry;
	psfp_tbl.sgit_entry = sgit_entry;
	psfp_tbl.isct_entry = isct_entry;
	psfp_tbl.sgclt_entry = sgclt_entry;
	err = netc_psfp_set_related_tables(priv, &psfp_tbl);
	if (err)
		goto clear_isct_eid_bit;

	rule->lastused = jiffies;
	rule->isft_entry = no_free_ptr(isft_entry);
	rule->isct_eid = isct_eid;

	if (reused_key_tbl) {
		rule->key_tbl = reused_key_tbl;
		refcount_inc(&reused_key_tbl->refcount);
	} else {
		rule->key_tbl = key_tbl;
	}

	if (reused_gate_tbl) {
		rule->gate_tbl = reused_gate_tbl;
		refcount_inc(&reused_gate_tbl->refcount);
	} else {
		gate_tbl->sgit_entry = no_free_ptr(sgit_entry);
		gate_tbl->sgclt_entry = no_free_ptr(sgclt_entry);
		refcount_set(&gate_tbl->refcount, 1);
		rule->gate_tbl = no_free_ptr(gate_tbl);
	}

	if (reused_police_tbl) {
		rule->police_tbl = reused_police_tbl;
		refcount_inc(&reused_police_tbl->refcount);
	} else if (police_tbl) {
		police_tbl->rpt_entry = no_free_ptr(rpt_entry);
		refcount_set(&police_tbl->refcount, 1);
		rule->police_tbl = no_free_ptr(police_tbl);
	}

	hlist_add_head(&no_free_ptr(rule)->node, &priv->flower_list);

	return 0;

clear_isct_eid_bit:
	ntmp_clear_eid_bitmap(priv->isct_eid_bitmap, isct_eid);

clear_sgclt_eid_words:
	if (sgclt_entry_size)
		ntmp_clear_words_bitmap(priv->sgclt_word_bitmap, sgclt_eid,
					sgclt_entry_size);

free_psfp_key_tbl:
	netc_free_flower_key_tbl(priv, key_tbl);

clear_rpt_eid_bit:
	if (police_entry && !reused_police_tbl)
		ntmp_clear_eid_bitmap(priv->rpt_eid_bitmap,
				      police_entry->hw_index);

clear_sgit_eid_bit:
	if (!reused_gate_tbl)
		ntmp_clear_eid_bitmap(priv->sgit_eid_bitmap,
				      gate_entry->hw_index);

	return err;
}
EXPORT_SYMBOL_GPL(netc_setup_psfp);

void netc_free_flower_police_tbl(struct ntmp_priv *priv,
				 struct netc_police_tbl *police_tbl)
{
	struct netc_cbdrs *cbdrs = &priv->cbdrs;

	if (!police_tbl)
		return;

	if (refcount_dec_and_test(&police_tbl->refcount)) {
		struct ntmp_rpt_entry *rpt_entry = police_tbl->rpt_entry;

		ntmp_rpt_delete_entry(cbdrs, rpt_entry->entry_id);
		ntmp_clear_eid_bitmap(priv->rpt_eid_bitmap,
				      rpt_entry->entry_id);
		kfree(rpt_entry);
		kfree(police_tbl);
	}
}
EXPORT_SYMBOL_GPL(netc_free_flower_police_tbl);

void netc_delete_psfp_flower_rule(struct ntmp_priv *priv,
				  struct netc_flower_rule *rule)
{
	struct ntmp_isft_entry *isft_entry = rule->isft_entry;
	struct netc_flower_key_tbl *key_tbl = rule->key_tbl;
	struct netc_gate_tbl *gate_tbl = rule->gate_tbl;
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	struct ntmp_isit_entry *isit_entry;
	struct ntmp_ist_entry *ist_entry;

	if (refcount_dec_and_test(&key_tbl->refcount)) {
		isit_entry = key_tbl->isit_entry;
		ist_entry = key_tbl->ist_entry;
		ntmp_isit_delete_entry(cbdrs, isit_entry->entry_id);
		ntmp_ist_delete_entry(cbdrs, ist_entry->entry_id);
		netc_free_flower_key_tbl(priv, key_tbl);
	}

	if (isft_entry) {
		ntmp_isft_delete_entry(cbdrs, isft_entry->entry_id);
		kfree(isft_entry);
	}

	ntmp_isct_operate_entry(cbdrs, rule->isct_eid, NTMP_CMD_DELETE, NULL);
	ntmp_clear_eid_bitmap(priv->isct_eid_bitmap, rule->isct_eid);

	if (gate_tbl && refcount_dec_and_test(&gate_tbl->refcount)) {
		netc_delete_sgit_entry(priv, gate_tbl->sgit_entry->entry_id);
		kfree(gate_tbl->sgit_entry);
		kfree(gate_tbl->sgclt_entry);
		kfree(gate_tbl);
	}

	netc_free_flower_police_tbl(priv, rule->police_tbl);

	hlist_del(&rule->node);
	kfree(rule);
}
EXPORT_SYMBOL_GPL(netc_delete_psfp_flower_rule);

int netc_psfp_flower_stat(struct ntmp_priv *priv, struct netc_flower_rule *rule,
			  u64 *byte_cnt, u64 *pkt_cnt, u64 *drop_cnt)
{
	struct ntmp_ist_entry *ist_entry = rule->key_tbl->ist_entry;
	struct ntmp_isft_entry *isft_entry = rule->isft_entry;
	struct isct_stse_data stse = {0};
	u32 isct_eid, sg_drop_cnt;
	int err;

	if (isft_entry)
		isct_eid = le32_to_cpu(isft_entry->cfge.isc_eid);
	else
		isct_eid = le32_to_cpu(ist_entry->cfge.isc_eid);

	/* Query, followed by update will reset statistics */
	err = ntmp_isct_operate_entry(&priv->cbdrs, isct_eid,
				      NTMP_CMD_QU, &stse);
	if (err)
		return err;

	sg_drop_cnt = le32_to_cpu(stse.sg_drop_count);
	/* Workaround for ERR052134 on i.MX95 platform */
	if (priv->errata & NTMP_ERR052134) {
		u32 tmp;

		sg_drop_cnt >>= 9;

		tmp = le32_to_cpu(stse.resv3) & 0x1ff;
		sg_drop_cnt |= (tmp << 23);
	}

	*pkt_cnt = le32_to_cpu(stse.rx_count);
	*drop_cnt = le32_to_cpu(stse.msdu_drop_count) + sg_drop_cnt +
		    le32_to_cpu(stse.policer_drop_count);

	return 0;
}
EXPORT_SYMBOL_GPL(netc_psfp_flower_stat);

int netc_setup_taprio(struct ntmp_priv *priv, u32 entry_id,
		      struct tc_taprio_qopt_offload *f)
{
	struct tgst_cfge_data *cfge __free(kfree) = NULL;
	struct netlink_ext_ack *extack = f->extack;
	u64 base_time = f->base_time;
	u64 max_cycle_time;
	int i, err;
	u32 size;

	if (!priv->get_tgst_free_words) {
		NL_SET_ERR_MSG_MOD(extack, "get_tgst_free_words() is undefined");
		return -EINVAL;
	}

	max_cycle_time = f->cycle_time + f->cycle_time_extension;
	if (max_cycle_time > U32_MAX) {
		NL_SET_ERR_MSG_MOD(extack, "Max cycle time exceeds U32_MAX");
		return -EINVAL;
	}

	/* Delete the pending administrative control list if it exists */
	err = ntmp_tgst_delete_admin_gate_list(&priv->cbdrs, entry_id);
	if (err)
		return err;

	if (f->num_entries > priv->get_tgst_free_words(priv)) {
		NL_SET_ERR_MSG_MOD(extack, "TGST doesn't have enough free words");
		return -EINVAL;
	}

	size = struct_size(cfge, ge, f->num_entries);
	cfge = kzalloc(size, GFP_KERNEL);
	if (!cfge)
		return -ENOMEM;

	if (priv->adjust_base_time)
		base_time = priv->adjust_base_time(priv, base_time, f->cycle_time);

	cfge->admin_bt = cpu_to_le64(base_time);
	cfge->admin_ct = cpu_to_le32(f->cycle_time);
	cfge->admin_ct_ext = cpu_to_le32(f->cycle_time_extension);
	cfge->admin_cl_len = cpu_to_le16(f->num_entries);
	for (i = 0; i < f->num_entries; i++) {
		struct tc_taprio_sched_entry *temp_entry = &f->entries[i];

		switch (temp_entry->command) {
		case TC_TAPRIO_CMD_SET_GATES:
			cfge->ge[i].hr_cb = HR_CB_SET_GATES;
			break;
		case TC_TAPRIO_CMD_SET_AND_HOLD:
			cfge->ge[i].hr_cb = HR_CB_SET_AND_HOLD;
			break;
		case TC_TAPRIO_CMD_SET_AND_RELEASE:
			cfge->ge[i].hr_cb = HR_CB_SET_AND_RELEASE;
			break;
		default:
			return -EOPNOTSUPP;
		}

		cfge->ge[i].tc_state = temp_entry->gate_mask;
		cfge->ge[i].interval = cpu_to_le32(temp_entry->interval);
	}

	err = ntmp_tgst_update_admin_gate_list(&priv->cbdrs, entry_id, cfge);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Update control list failed");
		return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(netc_setup_taprio);

int netc_ipft_keye_construct(struct flow_rule *rule, int port_id,
			     u16 prio, struct ipft_keye_data *keye,
			     struct netlink_ext_ack *extack)
{
	u16 frm_attr_flags = 0, src_port = 0;
	u16 vlan_tci, vlan_tci_mask;
	__be16 eth_type = 0;

	keye->precedence = cpu_to_le16(prio);

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_ETH_ADDRS)) {
		struct flow_match_eth_addrs match = {0};

		flow_rule_match_eth_addrs(rule, &match);
		ether_addr_copy(keye->dmac, match.key->dst);
		ether_addr_copy(keye->dmac_mask, match.mask->dst);
		ether_addr_copy(keye->smac, match.key->src);
		ether_addr_copy(keye->smac_mask, match.mask->src);
	}

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_VLAN)) {
		struct flow_match_vlan match = {0};

		flow_rule_match_vlan(rule, &match);
		vlan_tci = match.key->vlan_id | match.key->vlan_dei << 12 |
			   match.key->vlan_priority << VLAN_PRIO_SHIFT;
		vlan_tci_mask = match.mask->vlan_id | match.mask->vlan_dei << 12 |
				match.mask->vlan_priority << VLAN_PRIO_SHIFT;
		keye->outer_vlan_tci = htons(vlan_tci);
		keye->outer_vlan_tci_mask = htons(vlan_tci_mask);
		frm_attr_flags |= IPFT_FAF_OVLAN;
	}

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_CVLAN)) {
		struct flow_match_vlan match = {0};

		flow_rule_match_vlan(rule, &match);
		vlan_tci = match.key->vlan_id | match.key->vlan_dei << 12 |
			   match.key->vlan_priority << VLAN_PRIO_SHIFT;
		vlan_tci_mask = match.mask->vlan_id | match.mask->vlan_dei << 12 |
				match.mask->vlan_priority << VLAN_PRIO_SHIFT;
		keye->inner_vlan_tci = htons(vlan_tci);
		keye->inner_vlan_tci_mask = htons(vlan_tci_mask);
		frm_attr_flags |= IPFT_FAF_IVLAN;
	}

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_BASIC)) {
		struct flow_match_basic match = {0};

		flow_rule_match_basic(rule, &match);
		if (match.mask->n_proto && ntohs(match.mask->n_proto) != 0xffff) {
			NL_SET_ERR_MSG_MOD(extack, "Ether type mask must be 0xFFFF");
			return -EINVAL;
		}

		eth_type = match.key->n_proto;
		keye->ethertype = match.key->n_proto;
		keye->ethertype_mask = match.mask->n_proto;
		keye->ip_protocol = match.key->ip_proto;
		keye->ip_protocol_mask = match.mask->ip_proto;
		if (match.mask->ip_proto == 0xff) {
			if (match.key->ip_proto == IPPROTO_TCP)
				frm_attr_flags |= FIELD_PREP(IPFT_FAF_L4_CODE,
							     IPFT_FAF_TCP_HDR);
			else if (match.key->ip_proto == IPPROTO_UDP)
				frm_attr_flags |= FIELD_PREP(IPFT_FAF_L4_CODE,
							     IPFT_FAF_UDP_HDR);
		}
	}

	if (ntohs(eth_type) == ETH_P_IP &&
	    flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_IPV4_ADDRS)) {
		struct flow_match_ipv4_addrs match = {0};

		flow_rule_match_ipv4_addrs(rule, &match);
		keye->ip_dst[3] = match.key->dst;
		keye->ip_dst_mask[3] = match.mask->dst;
		keye->ip_src[3] = match.key->src;
		keye->ip_src_mask[3] = match.mask->src;
		frm_attr_flags |= IPFT_FAF_IP_HDR;
	}

	if (ntohs(eth_type) == ETH_P_IPV6 &&
	    flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_IPV6_ADDRS)) {
		struct flow_match_ipv6_addrs match = {0};

		flow_rule_match_ipv6_addrs(rule, &match);
		memcpy(keye->ip_dst, &match.key->dst, sizeof(keye->ip_dst));
		memcpy(keye->ip_dst_mask, &match.mask->dst, sizeof(keye->ip_dst_mask));
		memcpy(keye->ip_src, &match.key->src, sizeof(keye->ip_src));
		memcpy(keye->ip_src_mask, &match.mask->src, sizeof(keye->ip_src_mask));
		frm_attr_flags |= IPFT_FAF_IP_HDR | IPFT_FAF_IP_VER6;
	}

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_PORTS)) {
		struct flow_match_ports match = {0};

		flow_rule_match_ports(rule, &match);
		keye->l4_src_port = match.key->src;
		keye->l4_src_port_mask = match.mask->src;
		keye->l4_dst_port = match.key->dst;
		keye->l4_dst_port_mask = match.mask->dst;
	}

	keye->frm_attr_flags = cpu_to_le16(frm_attr_flags);
	keye->frm_attr_flags_mask = keye->frm_attr_flags;

	/* For ENETC, the port_id must be less than 0 */
	if (port_id >= 0) {
		src_port |= FIELD_PREP(IPFT_SRC_PORT, port_id);
		src_port |= IPFT_SRC_PORT_MASK;
		keye->src_port = cpu_to_le16(src_port);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(netc_ipft_keye_construct);

static int netc_add_police_key_tbl(struct ntmp_priv *priv, u32 rpt_eid,
				   struct netc_flower_key_tbl **key_tbl,
				   struct ipft_keye_data *ipft_key)
{
	struct netc_flower_key_tbl *new_tbl __free(kfree);
	struct ntmp_ipft_entry *ipft_entry __free(kfree);
	struct ipft_cfge_data *ipft_cfge;
	u32 cfg;

	new_tbl = kzalloc(sizeof(*new_tbl), GFP_KERNEL);
	if (!new_tbl)
		return -ENOMEM;

	ipft_entry = kzalloc(sizeof(*ipft_entry), GFP_KERNEL);
	if (!ipft_entry)
		return -ENOMEM;

	ipft_cfge = &ipft_entry->cfge;
	ipft_entry->keye = *ipft_key;

	cfg = FIELD_PREP(IPFT_FLTFA, IPFT_FLTFA_PERMIT);
	cfg |= FIELD_PREP(IPFT_FLTA, IPFT_FLTA_RP);
	ipft_cfge->cfg = cpu_to_le32(cfg);
	ipft_cfge->flta_tgt = cpu_to_le32(rpt_eid);

	new_tbl->tbl_type = FLOWER_KEY_TBL_IPFT;
	refcount_set(&new_tbl->refcount, 1);
	new_tbl->ipft_entry = no_free_ptr(ipft_entry);
	*key_tbl = no_free_ptr(new_tbl);

	return 0;
}

static int netc_set_police_tables(struct ntmp_priv *priv,
				  struct ntmp_ipft_entry *ipft_entry,
				  struct ntmp_rpt_entry *rpt_entry)
{
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	int err;

	if (rpt_entry) {
		err = ntmp_rpt_add_or_update_entry(cbdrs, rpt_entry);
		if (err)
			return err;
	}

	err = ntmp_ipft_add_entry(cbdrs, &ipft_entry->entry_id, ipft_entry);
	if (err)
		goto delete_rpt_entry;

	return 0;

delete_rpt_entry:
	if (rpt_entry)
		ntmp_rpt_delete_entry(cbdrs, rpt_entry->entry_id);

	return err;
}

int netc_setup_police(struct ntmp_priv *priv, int port_id,
		      struct flow_cls_offload *f)
{
	struct flow_rule *cls_rule = flow_cls_offload_flow_rule(f);
	struct netc_police_tbl *police_tbl __free(kfree) = NULL;
	struct ntmp_rpt_entry *rpt_entry __free(kfree) = NULL;
	struct netc_flower_rule *rule __free(kfree) = NULL;
	struct netlink_ext_ack *extack = f->common.extack;
	struct netc_police_tbl *reused_police_tbl = NULL;
	struct ipft_keye_data *ipft_keye __free(kfree);
	struct flow_action_entry *police_act = NULL;
	struct netc_flower_key_tbl *key_tbl = NULL;
	struct flow_action_entry *action_entry;
	struct ntmp_ipft_entry *ipft_entry;
	struct netc_flower_rule *old_rule;
	unsigned long cookie = f->cookie;
	u16 prio = f->common.prio;
	int i, err;

	guard(mutex)(&priv->flower_lock);
	if (netc_find_flower_rule_by_cookie(priv, port_id, cookie)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Cannot add new rule with same cookie");
		return -EINVAL;
	}

	rule = kzalloc(sizeof(*rule), GFP_KERNEL);
	if (!rule)
		return -ENOMEM;

	rule->port_id = port_id;
	rule->cookie = cookie;
	rule->flower_type = FLOWER_TYPE_POLICE;
	rule->isct_eid = NTMP_NULL_ENTRY_ID;

	flow_action_for_each(i, action_entry, &cls_rule->action)
		if (action_entry->id == FLOW_ACTION_POLICE)
			police_act = action_entry;

	if (!police_act) {
		NL_SET_ERR_MSG_MOD(extack, "No police action");
		return -EINVAL;
	}

	ipft_keye = kzalloc(sizeof(*ipft_keye), GFP_KERNEL);
	if (!ipft_keye)
		return -ENOMEM;

	err = netc_ipft_keye_construct(cls_rule, port_id, prio,
				       ipft_keye, extack);
	if (err)
		return err;

	old_rule = netc_find_flower_rule_by_key(priv, FLOWER_KEY_TBL_IPFT,
						ipft_keye);
	if (old_rule) {
		NL_SET_ERR_MSG_MOD(extack,
				   "The IPFT key has been used by existing rule");
		return -EINVAL;
	}

	err = netc_police_entry_validate(priv, &cls_rule->action, police_act,
					 &reused_police_tbl, extack);
	if (err)
		return err;

	if (!reused_police_tbl) {
		police_tbl = kzalloc(sizeof(*police_tbl), GFP_KERNEL);
		if (!police_tbl) {
			err = -ENOMEM;
			goto clear_rpt_eid_bit;
		}

		rpt_entry = kzalloc(sizeof(*rpt_entry), GFP_KERNEL);
		if (!rpt_entry) {
			err = -ENOMEM;
			goto clear_rpt_eid_bit;
		}

		netc_rpt_entry_config(police_act, rpt_entry);
	}

	err = netc_add_police_key_tbl(priv, police_act->hw_index, &key_tbl,
				      ipft_keye);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Failed to add police key table");
		goto clear_rpt_eid_bit;
	}

	ipft_entry = key_tbl->ipft_entry;
	err = netc_set_police_tables(priv, ipft_entry, rpt_entry);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Failed to add police table entries");
		goto clear_rpt_eid_bit;
	}

	rule->lastused = jiffies;
	rule->key_tbl = key_tbl;

	if (reused_police_tbl) {
		rule->police_tbl = reused_police_tbl;
		refcount_inc(&reused_police_tbl->refcount);
	} else if (police_tbl) {
		police_tbl->rpt_entry = no_free_ptr(rpt_entry);
		refcount_set(&police_tbl->refcount, 1);
		rule->police_tbl = no_free_ptr(police_tbl);
	}

	hlist_add_head(&no_free_ptr(rule)->node, &priv->flower_list);

	return 0;

clear_rpt_eid_bit:
	if (!reused_police_tbl)
		ntmp_clear_eid_bitmap(priv->rpt_eid_bitmap, police_act->hw_index);

	return err;
}
EXPORT_SYMBOL_GPL(netc_setup_police);

void netc_delete_police_flower_rule(struct ntmp_priv *priv,
				    struct netc_flower_rule *rule)
{
	struct netc_police_tbl *police_tbl = rule->police_tbl;
	struct netc_flower_key_tbl *key_tbl = rule->key_tbl;
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	struct ntmp_ipft_entry *ipft_entry;

	ipft_entry = key_tbl->ipft_entry;
	ntmp_ipft_delete_entry(cbdrs, ipft_entry->entry_id);

	netc_free_flower_police_tbl(priv, police_tbl);
	netc_free_flower_key_tbl(priv, key_tbl);

	hlist_del(&rule->node);
	kfree(rule);
}
EXPORT_SYMBOL_GPL(netc_delete_police_flower_rule);

int netc_police_flower_stat(struct ntmp_priv *priv,
			    struct netc_flower_rule *rule,
			    u64 *pkt_cnt)
{
	struct ntmp_ipft_entry *ipft_entry = rule->key_tbl->ipft_entry;
	struct ntmp_ipft_entry *ipft_query __free(kfree) = NULL;
	int err;

	ipft_query = kzalloc(sizeof(*ipft_query), GFP_KERNEL);
	if (!ipft_query)
		return -ENOMEM;

	err = ntmp_ipft_query_entry(&priv->cbdrs, ipft_entry->entry_id,
				    true, ipft_query);
	if (err)
		return err;

	*pkt_cnt = le64_to_cpu(ipft_query->match_count);

	return 0;
}
EXPORT_SYMBOL_GPL(netc_police_flower_stat);

static int netc_restore_gate_table(struct ntmp_priv *priv,
				   struct netc_gate_tbl *gate_tbl)
{
	struct ntmp_sgclt_entry *sgclt_entry = gate_tbl->sgclt_entry;
	struct ntmp_sgit_entry *sgit_entry = gate_tbl->sgit_entry;
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	u32 cycle_time;
	u64 base_time;
	int err;

	if (gate_tbl->restored)
		return 0;

	err = ntmp_sgclt_add_entry(cbdrs, sgclt_entry);
	if (err)
		return err;

	if (priv->adjust_base_time) {
		cycle_time = le32_to_cpu(sgclt_entry->cfge.cycle_time);
		base_time = le64_to_cpu(sgit_entry->acfge.admin_base_time);
		base_time = priv->adjust_base_time(priv, base_time, cycle_time);
		sgit_entry->acfge.admin_base_time = cpu_to_le64(base_time);
	}

	err = ntmp_sgit_add_or_update_entry(cbdrs, sgit_entry);
	if (err)
		goto del_sgit_entry;

	gate_tbl->restored = true;

	return 0;

del_sgit_entry:
	ntmp_sgclt_delete_entry(cbdrs, sgclt_entry->entry_id);

	return err;
}

static void netc_remove_gate_table(struct ntmp_priv *priv,
				   struct netc_gate_tbl *gate_tbl)
{
	struct ntmp_sgclt_entry *sgclt_entry = gate_tbl->sgclt_entry;
	struct ntmp_sgit_entry *sgit_entry = gate_tbl->sgit_entry;
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	struct ntmp_sgit_entry null_entry = {};

	null_entry.acfge.admin_sgcl_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
	null_entry.entry_id = sgit_entry->entry_id;
	ntmp_sgit_add_or_update_entry(cbdrs, &null_entry);
	ntmp_sgit_delete_entry(cbdrs, sgit_entry->entry_id);
	ntmp_sgclt_delete_entry(cbdrs, sgclt_entry->entry_id);
	gate_tbl->restored = false;
}

static int netc_restore_police_table(struct ntmp_priv *priv,
				     struct netc_police_tbl *police_tbl)
{
	struct ntmp_rpt_entry *rpt_entry = police_tbl->rpt_entry;
	int err;

	if (police_tbl->restored)
		return 0;

	err = ntmp_rpt_add_or_update_entry(&priv->cbdrs, rpt_entry);
	if (err)
		return err;

	police_tbl->restored = true;

	return 0;
}

static void netc_remove_police_table(struct ntmp_priv *priv,
				     struct netc_police_tbl *police_tbl)
{
	struct ntmp_rpt_entry *rpt_entry = police_tbl->rpt_entry;

	ntmp_rpt_delete_entry(&priv->cbdrs, rpt_entry->entry_id);
	police_tbl->restored = false;
}

static int netc_restore_key_table(struct ntmp_priv *priv,
				  struct netc_flower_key_tbl *key_tbl)
{
	struct ntmp_ist_entry *ist_entry = key_tbl->ist_entry;
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	struct ntmp_ipft_entry *ipft_entry;
	int err;

	if (key_tbl->restored)
		return 0;

	if (ist_entry) {
		err = ntmp_ist_add_or_update_entry(cbdrs, ist_entry);
		if (err)
			return err;
	}

	switch (key_tbl->tbl_type) {
	case FLOWER_KEY_TBL_ISIT:
		err = ntmp_isit_add_or_update_entry(cbdrs, true,
						    key_tbl->isit_entry);
		if (err)
			goto del_ist_entry;
		break;
	case FLOWER_KEY_TBL_IPFT:
		ipft_entry = key_tbl->ipft_entry;
		err = ntmp_ipft_add_entry(cbdrs, &ipft_entry->entry_id,
					  ipft_entry);
		if (err)
			goto del_ist_entry;
		break;
	}

	key_tbl->restored = true;

	return 0;

del_ist_entry:
	if (ist_entry)
		ntmp_ist_delete_entry(&priv->cbdrs, ist_entry->entry_id);

	return err;
}

static void netc_remove_key_table(struct ntmp_priv *priv,
				  struct netc_flower_key_tbl *key_tbl)
{
	struct ntmp_ist_entry *ist_entry = key_tbl->ist_entry;
	struct netc_cbdrs *cbdrs = &priv->cbdrs;

	switch (key_tbl->tbl_type) {
	case FLOWER_KEY_TBL_ISIT:
		ntmp_isit_delete_entry(cbdrs, key_tbl->isit_entry->entry_id);
		break;
	case FLOWER_KEY_TBL_IPFT:
		ntmp_ipft_delete_entry(cbdrs, key_tbl->ipft_entry->entry_id);
		break;
	}

	if (ist_entry)
		ntmp_ist_delete_entry(&priv->cbdrs, ist_entry->entry_id);

	key_tbl->restored = false;
}

static int netc_restore_flower_tables(struct ntmp_priv *priv,
				      struct netc_flower_rule *rule)
{
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	int err;

	if (rule->isct_eid != NTMP_NULL_ENTRY_ID) {
		err = ntmp_isct_operate_entry(cbdrs, rule->isct_eid,
					      NTMP_CMD_ADD, NULL);
		if (err)
			return err;
	}

	if (rule->gate_tbl) {
		err = netc_restore_gate_table(priv, rule->gate_tbl);
		if (err)
			goto del_isct_entry;
	}

	if (rule->police_tbl) {
		err = netc_restore_police_table(priv, rule->police_tbl);
		if (err)
			goto del_gate_table;
	}

	if (rule->isft_entry) {
		err = ntmp_isft_add_or_update_entry(cbdrs, true,
						    rule->isft_entry);
		if (err)
			goto del_police_table;
	}

	err = netc_restore_key_table(priv, rule->key_tbl);
	if (err)
		goto del_isft_entry;

	return 0;

del_isft_entry:
	if (rule->isft_entry)
		ntmp_isft_delete_entry(cbdrs, rule->isft_entry->entry_id);
del_police_table:
	if (rule->police_tbl)
		netc_remove_police_table(priv, rule->police_tbl);
del_gate_table:
	if (rule->gate_tbl)
		netc_remove_gate_table(priv, rule->gate_tbl);
del_isct_entry:
	if (rule->isct_eid != NTMP_NULL_ENTRY_ID)
		ntmp_isct_operate_entry(cbdrs, rule->isct_eid,
					NTMP_CMD_DELETE, NULL);

	return err;
}

static void netc_remove_flower_tables(struct ntmp_priv *priv,
				      struct netc_flower_rule *rule)
{
	struct ntmp_isft_entry *isft_entry = rule->isft_entry;
	struct netc_cbdrs *cbdrs = &priv->cbdrs;

	netc_remove_key_table(priv, rule->key_tbl);

	if (isft_entry)
		ntmp_isft_delete_entry(cbdrs, isft_entry->entry_id);

	if (rule->police_tbl)
		netc_remove_police_table(priv, rule->police_tbl);

	if (rule->gate_tbl)
		netc_remove_gate_table(priv, rule->gate_tbl);

	if (rule->isct_eid != NTMP_NULL_ENTRY_ID)
		ntmp_isct_operate_entry(cbdrs, rule->isct_eid,
					NTMP_CMD_DELETE, NULL);
}

static void netc_free_flower_rule(struct ntmp_priv *priv,
				  struct netc_flower_rule *rule)
{
	struct netc_police_tbl *police_tbl = rule->police_tbl;
	struct netc_flower_key_tbl *key_tbl = rule->key_tbl;
	struct netc_gate_tbl *gate_tbl = rule->gate_tbl;

	if (refcount_dec_and_test(&key_tbl->refcount))
		netc_free_flower_key_tbl(priv, key_tbl);

	kfree(rule->isft_entry);

	if (gate_tbl && refcount_dec_and_test(&gate_tbl->refcount)) {
		kfree(gate_tbl->sgit_entry);
		kfree(gate_tbl->sgclt_entry);
		kfree(gate_tbl);
	}

	if (police_tbl && refcount_dec_and_test(&police_tbl->refcount)) {
		kfree(police_tbl->rpt_entry);
		kfree(police_tbl);
	}

	hlist_del(&rule->node);
	kfree(rule);
}

int netc_restore_flower_list_config(struct ntmp_priv *priv)
{
	struct netc_flower_rule *rule, *iterator;
	struct hlist_node *tmp;
	int err;

	mutex_lock(&priv->flower_lock);

	hlist_for_each_entry(rule, &priv->flower_list, node) {
		err = netc_restore_flower_tables(priv, rule);
		if (err)
			goto del_flower_tables;
	}

	mutex_unlock(&priv->flower_lock);

	return 0;

del_flower_tables:
	hlist_for_each_entry(iterator, &priv->flower_list, node) {
		if (iterator == rule)
			break;

		netc_remove_flower_tables(priv, iterator);
	}

	hlist_for_each_entry_safe(iterator, tmp, &priv->flower_list, node)
		netc_free_flower_rule(priv, iterator);

	mutex_unlock(&priv->flower_lock);

	return err;
}
EXPORT_SYMBOL_GPL(netc_restore_flower_list_config);

void netc_clear_flower_table_restored_flag(struct ntmp_priv *priv)
{
	struct netc_flower_rule *rule;

	mutex_lock(&priv->flower_lock);

	hlist_for_each_entry(rule, &priv->flower_list, node) {
		rule->key_tbl->restored = false;

		if (rule->gate_tbl)
			rule->gate_tbl->restored = false;

		if (rule->police_tbl)
			rule->police_tbl->restored = false;
	}

	mutex_unlock(&priv->flower_lock);
}
EXPORT_SYMBOL_GPL(netc_clear_flower_table_restored_flag);
