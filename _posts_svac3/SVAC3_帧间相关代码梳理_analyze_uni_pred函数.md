---
layout:     post
title:      SVAC3_帧间相关代码梳理_analyze_uni_pred函数
subtitle:   v1.0版
date:       2023-12-25
author:     Shushi Chen
header-img: img/post-bg-cook.jpg
catalog: true
tags:
    - SVAC3/AVS3 参考模型阅读笔记
---

![]({{site.baseurl}}/img/logo.png)


# 前言
SVAC3标准是在AVS3标准上开发的，其代码结构与AVS3对应的代码结构相似，下面内容参考了AVS3开源编码器器“天枢”的相关文档，这里主要记录CU帧间编码相关代码结构。以下是_analyze_uni_pred函数
# analyze_uni_pred
```javascript
static void analyze_uni_pred(ENC_CTX *ctx, ENC_CORE *core, double *cost_L0L1, s16 mv_L0L1[REFP_NUM][MV_D], s8 *refi_L0L1, double *cost_best)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    ENC_PINTER *pi = &ctx->pinter;
    int lidx;
    s16 *mvp, *mv, *mvd;
#if FAST_LD
    u32 me_cost_l0[MAX_NUM_ACTIVE_REF_FRAME];      // Store temp L0 ME value
    int satd_decision[MAX_NUM_ACTIVE_REF_FRAME];  // Store temp LO and L1 SATD decision
#endif
    u32 mecost, best_mecost;
    s8 refi_cur = 0;
    s8 best_refi = 0;
    s8 t0, t1;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;

    mod_info_curr->cu_mode = MODE_INTER;
#if USE_IBC
    mod_info_curr->ibc_flag = 0;
#endif
#if USE_SP
    mod_info_curr->sp_flag = 0;
    mod_info_curr->cs2_flag = 0;
#endif
#if FAST_EXT_AMVR_HMVP
    mod_info_curr->mvp_from_hmvp_flag = 0;
    pi->mvp_from_hmvp_flag = 0;
#endif
#if UNIFIED_HMVP_1
    mod_info_curr->mvap_flag = 0;
    mod_info_curr->sub_tmvp_flag = 0;
#endif
    for (lidx = 0; lidx <= ((pi->slice_type == SLICE_P) ? PRED_L0 : PRED_L1); lidx++) // uni-prediction (L0 or L1)
    {
        init_inter_data(pi, core);
        mv = mod_info_curr->mv[lidx];
        mvd = mod_info_curr->mvd[lidx];
        pi->num_refp = (u8)ctx->rpm.num_refp[lidx];
        best_mecost = COM_UINT32_MAX;
        for (refi_cur = 0; refi_cur < pi->num_refp; refi_cur++)
        {
            mvp = pi->mvp_scale[lidx][refi_cur];

#if FAST_EXT_AMVR_HMVP
            if (ctx->info.sqh.emvr_enable_flag)
            {
                derive_mvp_info(ctx, core, lidx, refi_cur);
            }
            else
#endif
            // com_derive_mvp 里面是 MVP的构造过程，还没仔细阅读该函数，目前看下来这个MVP构造理念与264有些类似，是取中值的方法。2022_ICCE_A_Parallel_and_Pipelined_Hardware_Architecture_for_Fractional-Pixel_Motion_Estimation_in_AVS3 这篇文章也可以侧面印证。
            com_derive_mvp(ctx->info, mod_info_curr, ctx->ptr, lidx, refi_cur,   core->cnt_hmvp_cands,                                                
                core->motion_cands, ctx->map, ctx->refp, pi->curr_mvr, mvp);
#if MULTI_LAYER_FRAMEWORK
            if (ctx->info.pic_header.layer_id && !ctx->info.sqh.sps_independent_layer_flag[ctx->info.pic_header.layer_id] && pi->refp[refi_cur][lidx].pic->is_inter_layer == 1)
            {
                COM_PIC* ref_pic = ctx->refp[refi_cur][lidx].pic;
                pel* ref = ref_pic->y + x + y * ref_pic->stride_luma;
                int s_org = pi->stride_org[Y_C];
                pel* org = pi->Yuv_org[Y_C] + x + y * pi->stride_org[Y_C];

                mv[MV_X] = 0;
                mv[MV_Y] = 0;

                int mv_bits = get_mv_bits_with_mvr(mv[MV_X] - mvp[MV_X], mv[MV_Y] - mvp[MV_Y], pi->num_refp, refi_cur, pi->curr_mvr) + 1;
                mecost = MV_COST(pi, mv_bits);
                mecost += calc_satd_16b(cu_width, cu_height, org, ref, s_org, ref_pic->stride_luma, ctx->info.bit_depth_internal);

#if FAST_EXT_AMVR_HMVP
                check_best_uni_mvp(ctx, core, lidx, refi_cur, mv, &mecost);
#endif
                if (ctx->param.fast_ld_me && lidx == PRED_L0)
                {
                    me_cost_l0[refi_cur] = mecost;
                }
            }
            else
#endif
            // motion search
#if FAST_LD
            if (ctx->param.fast_ld_me && (lidx == PRED_L1 && ctx->info.pic_header.l1idx_to_l0idx[refi_cur] >= 0))
            {
                mv[MV_X] = pi->mv_scale[REFP_0][ctx->info.pic_header.l1idx_to_l0idx[refi_cur]][MV_X];
                mv[MV_Y] = pi->mv_scale[REFP_0][ctx->info.pic_header.l1idx_to_l0idx[refi_cur]][MV_Y];

                mecost = me_cost_l0[ctx->info.pic_header.l1idx_to_l0idx[refi_cur]];
                // refine mvd cost
                int mv_bits1 = get_mv_bits_with_mvr(mv[MV_X] - pi->mvp_scale[REFP_0][ctx->info.pic_header.l1idx_to_l0idx[refi_cur]][MV_X],
                    mv[MV_Y] - pi->mvp_scale[REFP_0][ctx->info.pic_header.l1idx_to_l0idx[refi_cur]][MV_Y],
                    ctx->rpm.num_refp[REFP_0], ctx->info.pic_header.l1idx_to_l0idx[refi_cur], pi->curr_mvr);

                mecost -= MV_COST(pi, mv_bits1);

                int mv_bits2 = get_mv_bits_with_mvr(mv[MV_X] - pi->mvp_scale[lidx][refi_cur][MV_X],
                    mv[MV_Y] - pi->mvp_scale[lidx][refi_cur][MV_Y],
                    pi->num_refp, refi_cur, pi->curr_mvr);

                mecost += MV_COST(pi, mv_bits2);

                if (mv_bits2 < mv_bits1)
                {
                    satd_decision[refi_cur] = 1;
                }
                else
                {
                    satd_decision[refi_cur] = 0;
                }
            }
            else
            {
#if INTER_ME_MVLIB
#if ENC_ME_IMP
                // 执行ME的函数，LDP下执行的是epzs，分像素运动估计也在这个过程中
                mecost = pi->fn_me(ctx, pi, core, x, y, cu_width, cu_height, mod_info_curr->x_pos, mod_info_curr->y_pos, cu_width, &refi_cur, lidx, mvp, mv, 0
#if CU_LEVEL_PRIVACY
                    , mod_info_curr->cu_level
#endif
                );
#else
                mecost = pi->fn_me(pi, core, x, y, cu_width, cu_height, mod_info_curr->x_pos, mod_info_curr->y_pos, cu_width, &refi_cur, lidx, mvp, mv, 0);
#endif
#else
#if ENC_ME_IMP
                mecost = pi->fn_me(ctx, pi, x, y, cu_width, cu_height, mod_info_curr->x_pos, mod_info_curr->y_pos, cu_width, &refi_cur, lidx, mvp, mv, 0);
#else
                mecost = pi->fn_me(pi, x, y, cu_width, cu_height, mod_info_curr->x_pos, mod_info_curr->y_pos, cu_width, &refi_cur, lidx, mvp, mv, 0);
#endif
#endif
#if FAST_EXT_AMVR_HMVP
                check_best_uni_mvp(ctx, core, lidx, refi_cur, mv, &mecost);
#endif
                if (ctx->param.fast_ld_me && lidx == PRED_L0)
                {
                    me_cost_l0[refi_cur] = mecost;
                }
            }
#else
#if INTER_ME_MVLIB
#if ENC_ME_IMP
            mecost = pi->fn_me(ctx, pi, core, x, y, cu_width, cu_height, mod_info_curr->x_pos, mod_info_curr->y_pos, cu_width, &refi_cur, lidx, mvp, mv, 0);
#else
            mecost = pi->fn_me(pi, core, x, y, cu_width, cu_height, mod_info_curr->x_pos, mod_info_curr->y_pos, cu_width, &refi_cur, lidx, mvp, mv, 0);
#endif
#else
#if ENC_ME_IMP
            mecost = pi->fn_me(ctx, pi, x, y, cu_width, cu_height, mod_info_curr->x_pos, mod_info_curr->y_pos, cu_width, &refi_cur, lidx, mvp, mv, 0);
#else
            mecost = pi->fn_me(pi, x, y, cu_width, cu_height, mod_info_curr->x_pos, mod_info_curr->y_pos, cu_width, &refi_cur, lidx, mvp, mv, 0);
#endif
#endif
#endif
            pi->mv_scale[lidx][refi_cur][MV_X] = mv[MV_X];
            pi->mv_scale[lidx][refi_cur][MV_Y] = mv[MV_Y];
            if (mecost < best_mecost)
            {
                best_mecost = mecost;
                best_refi = refi_cur;
            }

#if BD_AFFINE_AMVR
            if (pi->curr_mvr < MAX_NUM_AFFINE_MVR)
#else
            if (pi->curr_mvr == 0)
#endif
            {
                pi->best_mv_uni[lidx][refi_cur][MV_X] = mv[MV_X];
                pi->best_mv_uni[lidx][refi_cur][MV_Y] = mv[MV_Y];
            }
        }
        mv[MV_X] = pi->mv_scale[lidx][best_refi][MV_X];
        mv[MV_Y] = pi->mv_scale[lidx][best_refi][MV_Y];
        mvp = pi->mvp_scale[lidx][best_refi];
#if FAST_EXT_AMVR_HMVP
        pi->mvp_from_hmvp_flag = pi->mvp_flag[lidx][best_refi];
        mod_info_curr->mvp_from_hmvp_flag = pi->mvp_flag[lidx][best_refi];
#endif
        t0 = (lidx == 0) ? best_refi : REFI_INVALID;
        t1 = (lidx == 1) ? best_refi : REFI_INVALID;
        SET_REFI(mod_info_curr->refi, t0, t1);
        refi_L0L1[lidx] = best_refi;

        mv[MV_X] = (mv[MV_X] >> pi->curr_mvr) << pi->curr_mvr;
        mv[MV_Y] = (mv[MV_Y] >> pi->curr_mvr) << pi->curr_mvr;

        mvd[MV_X] = mv[MV_X] - mvp[MV_X];
        mvd[MV_Y] = mv[MV_Y] - mvp[MV_Y];

        /* important: reset mv/mvd */
        {
            // note: reset mvd, after clipping, mvd might not align with amvr index
            int amvr_shift = pi->curr_mvr;
            mvd[MV_X] = mvd[MV_X] >> amvr_shift << amvr_shift;
            mvd[MV_Y] = mvd[MV_Y] >> amvr_shift << amvr_shift;

            // note: reset mv, after clipping, mv might not equal to mvp + mvd
            int mv_x = (s32)mvd[MV_X] + mvp[MV_X];
            int mv_y = (s32)mvd[MV_Y] + mvp[MV_Y];
            mv[MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_x);
            mv[MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_y);
        }

        mv_L0L1[lidx][MV_X] = mv[MV_X];
        mv_L0L1[lidx][MV_Y] = mv[MV_Y];

        // get mot_bits for bi search: mvd + refi
        pi->mot_bits[lidx] = get_mv_bits_with_mvr(mvd[MV_X], mvd[MV_Y], pi->num_refp, best_refi, pi->curr_mvr);
        pi->mot_bits[lidx] -= (pi->curr_mvr == MAX_NUM_MVR - 1) ? pi->curr_mvr : (pi->curr_mvr + 1); // minus amvr index
#if INTERPF
        core->mod_info_curr.inter_filter_flag = 0;
#endif
#if IPC
        core->mod_info_curr.ipc_flag = 0;
#endif

#if FAST_LD
        if (ctx->param.fast_ld_me && lidx == PRED_L1 && refi_L0L1[REFP_0] == ctx->info.pic_header.l1idx_to_l0idx[refi_L0L1[REFP_1]] && satd_decision[refi_L0L1[REFP_1]] == 0)
        {
            continue;
        }
        else
        {
            cost_L0L1[lidx] = pinter_residue_rdo(ctx, core, 0
#if DMVR
                , 0
#endif
            );

            check_best_mode(core, pi, cost_L0L1[lidx], cost_best);
        }
#else
        cost_L0L1[lidx] = pinter_residue_rdo(ctx, core, 0
#if DMVR
                                             , 0
#endif
        );
        check_best_mode(core, pi, cost_L0L1[lidx], cost_best);
#endif
    }
#if FAST_EXT_AMVR_HMVP
    if (ctx->info.sqh.emvr_enable_flag)
    {
        pi->mvp_from_hmvp_flag = core->mod_info_best.mvp_from_hmvp_flag;
        mod_info_curr->mvp_from_hmvp_flag = core->mod_info_best.mvp_from_hmvp_flag;
        memcpy(pi->mvp_scale, pi->mvps_uni[core->mod_info_best.mvp_from_hmvp_flag], REFP_NUM * MAX_NUM_ACTIVE_REF_FRAME * MV_D * sizeof(s16));
    }
#endif
#if INTER_ME_MVLIB
    if (pi->mvp_from_hmvp_flag == 0)
    {
        ENC_ME_MVLIB *mvlib = &core->s_enc_me_mvlib[pi->curr_mvr];
        insertUniMvCands(mvlib, x, y, cu_width, cu_height, pi->mv_scale);
        memcpy(mvlib->uni_mv_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].reused_uni_mv, pi->mv_scale, REFP_NUM * MAX_NUM_ACTIVE_REF_FRAME * MV_D * sizeof(s16));
        mvlib->uni_mv_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].visit = 1;
    }
#endif
}
```

# analyze_uni_pred

```javascript

#if ENC_ME_IMP
static u32 pinter_me_epzs(ENC_CTX * ctx, ENC_PINTER * pi, ENC_CORE * core, int x, int y, int w, int h, int cu_x, int cu_y, int cu_stride, s8 * refi, int lidx, s16 mvp[MV_D], s16 mv[MV_D], int bi
#if CU_LEVEL_PRIVACY
    , int cu_flag
#endif
)
#else
static u32 pinter_me_epzs(ENC_PINTER * pi, ENC_CORE * core, int x, int y, int w, int h, int cu_x, int cu_y, int cu_stride, s8 * refi, int lidx, s16 mvp[MV_D], s16 mv[MV_D], int bi)
#endif
#else
#if ENC_ME_IMP
static u32 pinter_me_epzs(ENC_CTX * ctx, ENC_PINTER * pi, int x, int y, int w, int h, int cu_x, int cu_y, int cu_stride, s8 * refi, int lidx, s16 mvp[MV_D], s16 mv[MV_D], int bi)
#else
static u32 pinter_me_epzs(ENC_PINTER * pi, int x, int y, int w, int h, int cu_x, int cu_y, int cu_stride, s8 * refi, int lidx, s16 mvp[MV_D], s16 mv[MV_D], int bi)
#endif
#endif
{
    s16 mvc[MV_D];  /* MV center for search */
    s16 gmvp[MV_D]; /* MVP in frame coordinate */
    s16 range[MV_RANGE_DIM][MV_D]; /* search range after clipping */
    s16 mvi[MV_D];
    s16 mvt[MV_D];
    u32 cost, cost_best = COM_UINT32_MAX;
    s8 ref_idx = *refi;  /* reference buffer index */
    int tmpstep = 0;
    int beststep = 0;
    gmvp[MV_X] = mvp[MV_X] + ((s16)x << 2);
    gmvp[MV_Y] = mvp[MV_Y] + ((s16)y << 2);
#if INTER_ME_MVLIB
    COM_PIC *ref_pic = pi->refp[*refi][lidx].pic;
#if OBMC
    pel     *org = pi->org_obmc + (x - cu_x) + (y - cu_y) * cu_stride;
#else
    pel     *org     = pi->Yuv_org[Y_C] + y * pi->stride_org[Y_C] + x;
#endif
    s16     *org_bi  = pi->org_bi + (x - cu_x) + (y - cu_y) * cu_stride;
    pel     *ref;
    s16     tmp_mv[MV_D];
#if OBMC
    u8     mvr_idx_list[5];
    u8     cand_idx_list[5];
    u32    cost_cand_list[5];
    if (ctx->info.sqh.obmc_enable_flag && !bi && (pi->curr_mvr < 2) && (pi->me_level > ME_LEV_IPEL))
    {
        for (int i = 0; i < 5; i++)
        {
            cost_cand_list[i] = COM_UINT32_MAX;
        }
    }
#endif
#endif

#if ENC_ME_IMP
    s16 best_grad_mv[MV_D];
#endif

    if (!bi && pi->mvp_from_hmvp_flag == 0 && pi->imv_valid[lidx][ref_idx] && pi->curr_mvr < 3)
    {
        mvi[MV_X] = pi->imv[lidx][ref_idx][MV_X] + ((s16)x << 2);
        mvi[MV_Y] = pi->imv[lidx][ref_idx][MV_Y] + ((s16)y << 2);
        mvc[MV_X] = (s16)x + (pi->imv[lidx][ref_idx][MV_X] >> 2);
        mvc[MV_Y] = (s16)y + (pi->imv[lidx][ref_idx][MV_Y] >> 2);
#if ENC_ME_IMP
        best_grad_mv[MV_X] = pi->imv[lidx][ref_idx][MV_X];
        best_grad_mv[MV_Y] = pi->imv[lidx][ref_idx][MV_Y];
#endif
    }
    else
    {
        if (bi)
        {
            mvi[MV_X] = mv[MV_X] + ((s16)x << 2);
            mvi[MV_Y] = mv[MV_Y] + ((s16)y << 2);
            mvc[MV_X] = (s16)x + (mv[MV_X] >> 2);
            mvc[MV_Y] = (s16)y + (mv[MV_Y] >> 2);
#if ENC_ME_IMP
            best_grad_mv[MV_X] = mv[MV_X];
            best_grad_mv[MV_Y] = mv[MV_Y];
#endif
        }
        else
        {
            mvi[MV_X] = mvp[MV_X] + ((s16)x << 2);
            mvi[MV_Y] = mvp[MV_Y] + ((s16)y << 2);
            mvc[MV_X] = (s16)x + (mvp[MV_X] >> 2);
            mvc[MV_Y] = (s16)y + (mvp[MV_Y] >> 2);
#if ENC_ME_IMP
            best_grad_mv[MV_X] = mvp[MV_X];
            best_grad_mv[MV_Y] = mvp[MV_Y];
#endif
        }
    }

    ref_idx = *refi;
    mvc[MV_X] = COM_CLIP3(pi->min_mv_offset[MV_X], pi->max_mv_offset[MV_X], mvc[MV_X]);
    mvc[MV_Y] = COM_CLIP3(pi->min_mv_offset[MV_Y], pi->max_mv_offset[MV_Y], mvc[MV_Y]);
#if INTER_ME_MVLIB
    if (pi->curr_mvr > 2)
    {
        com_mv_rounding_s16(mvc[MV_X], mvc[MV_Y], &mvc[MV_X], &mvc[MV_Y], pi->curr_mvr - 2, pi->curr_mvr - 2);
    }
    int mv_bits = get_mv_bits_with_mvr((mvc[MV_X] << 2) - gmvp[MV_X], (mvc[MV_Y] << 2) - gmvp[MV_Y], pi->num_refp, ref_idx, pi->curr_mvr);
    u32 initCost = MV_COST(pi, mv_bits);
    ref = ref_pic->y + mvc[MV_X] + mvc[MV_Y] * ref_pic->stride_luma;
    if (bi)
    {
        /* get sad */
        initCost += calc_sad_16b(w, h, org_bi, ref, cu_stride, ref_pic->stride_luma, pi->bit_depth) >> 1;
    }
    else
    {
        /* get sad */
#if OBMC
        initCost += calc_sad_16b(w, h, org, ref, cu_stride, ref_pic->stride_luma, pi->bit_depth);
#else
        initCost += calc_sad_16b(w, h, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
#endif
    }

    for (int mvr = 0; mvr < MAX_NUM_MVR; mvr++)
    {
    if (!((ctx->info.sqh.obmc_enable_flag && !bi && (pi->curr_mvr < 2) && (pi->me_level > ME_LEV_IPEL)) || (mvr == pi->curr_mvr)))
    {
        continue;
    }
    ENC_ME_MVLIB *ptr = &core->s_enc_me_mvlib[mvr];
    for (int i = 0; i < ptr->uni_mv_list_size; i++)
    {
        BLK_UNI_MV_INFO* curMvInfo = ptr->uni_mv_list + ((ptr->uni_mv_list_idx - 1 - i + ptr->uni_mv_list_max_size) % (ptr->uni_mv_list_max_size));

        int j = 0;
        for (; j < i; j++)
        {
            BLK_UNI_MV_INFO *prevMvInfo = ptr->uni_mv_list + ((ptr->uni_mv_list_idx - 1 - j + ptr->uni_mv_list_max_size) % (ptr->uni_mv_list_max_size));
            if (SAME_MV(curMvInfo->uniMvs[lidx][ref_idx], prevMvInfo->uniMvs[lidx][ref_idx]))
            {
                break;
            }
        }
        if (j < i)
        {
            continue;
        }

        tmp_mv[MV_X] = (s16)x + (curMvInfo->uniMvs[lidx][ref_idx][MV_X] >> 2);
        tmp_mv[MV_Y] = (s16)y + (curMvInfo->uniMvs[lidx][ref_idx][MV_Y] >> 2);
        tmp_mv[MV_X] = COM_CLIP3(pi->min_mv_offset[MV_X], pi->max_mv_offset[MV_X], tmp_mv[MV_X]);
        tmp_mv[MV_Y] = COM_CLIP3(pi->min_mv_offset[MV_Y], pi->max_mv_offset[MV_Y], tmp_mv[MV_Y]);
        if (pi->curr_mvr > 2)
        {
            com_mv_rounding_s16(tmp_mv[MV_X], tmp_mv[MV_Y], &tmp_mv[MV_X], &tmp_mv[MV_Y], pi->curr_mvr - 2, pi->curr_mvr - 2);
        }
        mv_bits = get_mv_bits_with_mvr((tmp_mv[MV_X] << 2) - gmvp[MV_X], (tmp_mv[MV_Y] << 2) - gmvp[MV_Y], pi->num_refp, ref_idx, pi->curr_mvr);
        u32 tmpCost = MV_COST(pi, mv_bits);
        ref = ref_pic->y + tmp_mv[MV_X] + tmp_mv[MV_Y] * ref_pic->stride_luma;
        if (bi)
        {
            /* get sad */
            tmpCost += calc_sad_16b(w, h, org_bi, ref, cu_stride, ref_pic->stride_luma, pi->bit_depth) >> 1;
        }
        else
        {
            /* get sad */
#if OBMC
            tmpCost += calc_sad_16b(w, h, org, ref, cu_stride, ref_pic->stride_luma, pi->bit_depth);
#else
            tmpCost += calc_sad_16b(w, h, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
#endif
        }

#if OBMC
        if (ctx->info.sqh.obmc_enable_flag && !bi && (pi->curr_mvr < 2) && (pi->me_level > ME_LEV_IPEL))
        {
            sort_unimv_list(mvr, (u8)i, tmpCost, mvr_idx_list, cand_idx_list, cost_cand_list);
        }
#endif
        if ((tmpCost < initCost) && (mvr == pi->curr_mvr))
        {
            initCost = tmpCost;
            mvc[MV_X] = tmp_mv[MV_X];
            mvc[MV_Y] = tmp_mv[MV_Y];
        }
    }
    }

    mvi[MV_X] = mvc[MV_X] << 2;
    mvi[MV_Y] = mvc[MV_Y] << 2;
#endif
    get_range_ipel(pi, mvc, range, ref_idx, lidx);
    cost = me_ipel_diamond(pi, x, y, w, h, cu_x, cu_y, cu_stride, ref_idx, lidx, range, gmvp, mvi, mvt, bi, &tmpstep, MAX_FIRST_SEARCH_STEP);
    if (cost < cost_best)
    {
        cost_best = cost;
        mv[MV_X] = mvt[MV_X];
        mv[MV_Y] = mvt[MV_Y];
        if (abs(mvp[MV_X] - mv[MV_X]) < 2 && abs(mvp[MV_Y] - mv[MV_Y]) < 2)
        {
            beststep = 0;
        }
        else
        {
            beststep = tmpstep;
        }
    }
    if (!bi && beststep > RASTER_SEARCH_THD)
    {
        cost = me_raster(pi, x, y, w, h, ref_idx, lidx, range, gmvp, mvt);
        if (cost < cost_best)
        {
            beststep = RASTER_SEARCH_THD;
            cost_best = cost;
            mv[MV_X] = mvt[MV_X];
            mv[MV_Y] = mvt[MV_Y];
        }
    }
    if (!bi && beststep > REFINE_SEARCH_THD)
    {
        mvc[MV_X] = (s16)x + (mv[MV_X] >> 2);
        mvc[MV_Y] = (s16)y + (mv[MV_Y] >> 2);
        get_range_ipel(pi, mvc, range, ref_idx, lidx);
        mvi[MV_X] = mv[MV_X] + ((s16)x << 2);
        mvi[MV_Y] = mv[MV_Y] + ((s16)y << 2);
        cost = me_ipel_diamond(pi, x, y, w, h, cu_x, cu_y, cu_stride, ref_idx, lidx, range, gmvp, mvi, mvt, bi, &tmpstep, MAX_REFINE_SEARCH_STEP);
        if (cost < cost_best)
        {
            cost_best = cost;
            mv[MV_X] = mvt[MV_X];
            mv[MV_Y] = mvt[MV_Y];
        }
    }

#if FAST_EXT_AMVR_HMVP
    if (!bi && pi->mvp_from_hmvp_flag == 0 && pi->imv_valid[lidx][ref_idx] == 0)
#else
    if (!bi && pi->imv_valid[lidx][ref_idx] == 0)
#endif
    {
        pi->imv[lidx][ref_idx][MV_X] = mv[MV_X];
        pi->imv[lidx][ref_idx][MV_Y] = mv[MV_Y];
        pi->imv_valid[lidx][ref_idx] = 1;
    }

    if (pi->me_level > ME_LEV_IPEL && (pi->curr_mvr == 0 || pi->curr_mvr == 1))
    {
        /* sub-pel ME */　// FME 过程
        cost = me_spel_pattern(pi, x, y, w, h, cu_x, cu_y, cu_stride, ref_idx, lidx, gmvp, mv, mvt, bi
#if CU_LEVEL_PRIVACY
            , cu_flag, ctx->info.pred_map
#endif
        );
        cost_best = cost;
        mv[MV_X] = mvt[MV_X];
        mv[MV_Y] = mvt[MV_Y];
    }

#if ENC_ME_IMP
#if OBMC
    BOOL testGradME = FALSE;
    if (!ctx->info.sqh.obmc_enable_flag || ctx->info.pic_header.is_lowdelay)
    {
        testGradME = (pi->curr_mvr < 1);
    }
    else
    {
        if (!bi)
        {
            testGradME = TRUE;
        }
        else
        {
            testGradME = (pi->curr_mvr < 1);
        }
    }
    if (testGradME)
#else
    if (pi->curr_mvr < 1)
#endif
    {
        if (!(ctx->info.pic_header.is_lowdelay && bi))
        {
            cost = me_grad_search(pi, x, y, w, h, cu_x, cu_y, cu_stride, ref_idx, lidx, gmvp, best_grad_mv, mvt, bi
#if CU_LEVEL_PRIVACY
                , cu_flag, ctx->info.pred_map
#endif
            );
            if (cost < cost_best)
            {
                cost_best = cost;
                mv[MV_X] = mvt[MV_X];
                mv[MV_Y] = mvt[MV_Y];
            }
        }
    }

#if OBMC
    if (ctx->info.sqh.obmc_enable_flag && !bi && (pi->curr_mvr < 2) && (pi->me_level > ME_LEV_IPEL))
    {
        int  pic_w = pi->pic_org->width_luma;
        int  pic_h = pi->pic_org->height_luma;
        s16  cur_mv[MV_D];

        pel* org = pi->org_obmc + (x - cu_x) + (y - cu_y) * cu_stride;
        int  s_org = cu_stride;
        pel* ref = pi->refp[ref_idx][lidx].pic->y;
        int  s_ref = pi->refp[ref_idx][lidx].pic->stride_luma;
        pel* pred = pi->pred_buf + (x - cu_x) + (y - cu_y) * cu_stride;
        int  s_pred = cu_stride;

        for (int i = 0; i < 5 && (cost_cand_list[i] != COM_UINT32_MAX); i++)
        {
            int idx = cand_idx_list[i];
            int mvr_idx = mvr_idx_list[i];
            ENC_ME_MVLIB *ptr = &core->s_enc_me_mvlib[mvr_idx];
            BLK_UNI_MV_INFO* curMvInfo = ptr->uni_mv_list + ((ptr->uni_mv_list_idx - 1 - idx + ptr->uni_mv_list_max_size) % (ptr->uni_mv_list_max_size));
            cur_mv[MV_X] = curMvInfo->uniMvs[lidx][ref_idx][MV_X];
            cur_mv[MV_Y] = curMvInfo->uniMvs[lidx][ref_idx][MV_Y];
            cur_mv[MV_X] = (cur_mv[MV_X] >> pi->curr_mvr) << pi->curr_mvr;
            cur_mv[MV_Y] = (cur_mv[MV_Y] >> pi->curr_mvr) << pi->curr_mvr;

            if ((cur_mv[MV_X] != mv[MV_X]) || (cur_mv[MV_Y] != mv[MV_Y]))
            {
                s16 mv_clip[MV_D];
                valid_mv_clip(x, y, pic_w, pic_h, w, h, cur_mv, mv_clip);
                s16 cx = cur_mv[MV_X] + ((s16)x << 2);
                s16 cy = cur_mv[MV_Y] + ((s16)y << 2);
                int bits = get_mv_bits_with_mvr(cx - gmvp[MV_X], cy - gmvp[MV_Y], pi->num_refp, ref_idx, pi->curr_mvr);
                bits += 2;
                cost = MV_COST(pi, bits);

                cx = mv_clip[MV_X] + ((s16)x << 2);
                cy = mv_clip[MV_Y] + ((s16)y << 2);

#if CU_LEVEL_PRIVACY
                com_mc_l(cur_mv[MV_X], cur_mv[MV_Y], ref, cx, cy, s_ref, cu_stride, pred, w, h, pi->bit_depth, pi->refp[ref_idx][lidx].pic_privacy, pi->refp[ref_idx][lidx].pic->width_luma>>MIN_CU_LOG2, pi->refp[ref_idx][lidx].pic->height_luma >> MIN_CU_LOG2, cu_flag, ctx->info.pred_map);
#else
                com_mc_l(cur_mv[MV_X], cur_mv[MV_Y], ref, cx, cy, s_ref, cu_stride, pred, w, h, pi->bit_depth);
#endif

                cost += calc_satd_16b(w, h, org, pred, s_org, s_pred, pi->bit_depth);

                if (cost < cost_best)
                {
                    cost_best = cost;
                    mv[MV_X] = cur_mv[MV_X];
                    mv[MV_Y] = cur_mv[MV_Y];
                }
            }
        }
    }
#endif

    if (!bi)
    {
        s16 cx = mv[MV_X] + ((s16)x << 2);
        s16 cy = mv[MV_Y] + ((s16)y << 2);
        s32 bits = get_mv_bits_with_mvr(cx - gmvp[MV_X], cy - gmvp[MV_Y], pi->num_refp, ref_idx, pi->curr_mvr);
        bits += 2;
        pi->mot_bits[lidx] = bits;
    }
#endif

    return cost_best;
}
```

# me_spel_pattern

```javascript

static u32 me_spel_pattern(ENC_PINTER *pi, int x, int y, int w, int h, int cu_x, int cu_y, int cu_stride, s8 refi, int lidx, s16 gmvp[MV_D], s16 mvi[MV_D], s16 mv[MV_D], int bi
#if CU_LEVEL_PRIVACY
    ,int cu_flag, pel *pred_map
#endif
 )
{
    int bit_depth = pi->bit_depth;
    pel     *org, *ref, *pred;
    s16     *org_bi;
    u32      cost, cost_best = COM_UINT32_MAX;
    s16      mv_x, mv_y, cx, cy;
    int      lidx_r = (lidx == REFP_0) ? REFP_1 : REFP_0;
    int      i, mv_bits, s_org, s_ref, best_mv_bits;
#if OBMC
    s_org = cu_stride;
    org = pi->org_obmc + (x - cu_x) + (y - cu_y) * cu_stride;
#else
    s_org = pi->stride_org[Y_C];
    org = pi->Yuv_org[Y_C] + x + y * pi->stride_org[Y_C];
#endif
    s_ref = pi->refp[refi][lidx].pic->stride_luma;
    ref = pi->refp[refi][lidx].pic->y;
    org_bi = pi->org_bi + (x - cu_x) + (y - cu_y) * cu_stride;
    pred = pi->pred_buf + (x - cu_x) + (y - cu_y) * cu_stride;
    best_mv_bits = 0;
    /* make MV to be global coordinate */
    cx = mvi[MV_X] + ((s16)x << 2);
    cy = mvi[MV_Y] + ((s16)y << 2);
    /* intial value */
    mv[MV_X] = mvi[MV_X];
    mv[MV_Y] = mvi[MV_Y];

    // get initial satd cost as cost_best
    mv_bits = get_mv_bits_with_mvr(cx - gmvp[MV_X], cy - gmvp[MV_Y], pi->num_refp, refi, pi->curr_mvr);
    mv_bits += bi ? 1 : 2; // add inter_dir bits
    if (bi)
    {
        mv_bits += pi->mot_bits[lidx_r];
    }
    /* get MVD cost_best */
    cost_best = MV_COST(pi, mv_bits);
    pel * ref_tmp = ref + (cx >> 2) + (cy >> 2)* s_ref;
#if CU_LEVEL_PRIVACY
    u8 *privacy_map = pi->refp[refi][lidx].pic_privacy;
    int map_s = pi->refp[refi][lidx].pic->width_luma >> MIN_CU_LOG2;
    int map_h = pi->refp[refi][lidx].pic->height_luma >> MIN_CU_LOG2;
#endif

    if (bi)
    {
        /* get satd */
        cost_best += calc_satd_16b(w, h, org_bi, ref_tmp, cu_stride, s_ref, bit_depth) >> 1;
    }
    else
    {
        /* get satd */
        cost_best += calc_satd_16b(w, h, org, ref_tmp, s_org, s_ref, bit_depth);
    }

    /* search upto hpel-level from here */
    /* search of large diamond pattern */
    for (i = 0; i < pi->search_pattern_hpel_cnt; i++)
    {
        mv_x = cx + pi->search_pattern_hpel[i][0];
        mv_y = cy + pi->search_pattern_hpel[i][1];
        /* get MVD bits */
        mv_bits = get_mv_bits_with_mvr(mv_x - gmvp[MV_X], mv_y - gmvp[MV_Y], pi->num_refp, refi, pi->curr_mvr);
        mv_bits += bi ? 1 : 2; // add inter_dir bits
        if (bi)
        {
            mv_bits += pi->mot_bits[lidx_r];
        }
        /* get MVD cost_best */
        cost = MV_COST(pi, mv_bits);
        /* get the interpolated(predicted) image */

#if CU_LEVEL_PRIVACY
        com_mc_l(mv_x, mv_y, ref, mv_x, mv_y, s_ref, cu_stride, pred, w, h, bit_depth, privacy_map, map_s,map_h,cu_flag, pred_map);
#else
        com_mc_l(mv_x, mv_y, ref, mv_x, mv_y, s_ref, cu_stride, pred, w, h, bit_depth);
#endif

        if (bi)
        {
            /* get sad */
            cost += calc_satd_16b(w, h, org_bi, pred, cu_stride, cu_stride, bit_depth) >> 1;
        }
        else
        {
            /* get sad */
            cost += calc_satd_16b(w, h, org, pred, s_org, cu_stride, bit_depth);
        }
        /* check if motion cost_best is less than minimum cost_best */
        if (cost < cost_best)
        {
            mv[MV_X] = mv_x - ((s16)x << 2);
            mv[MV_Y] = mv_y - ((s16)y << 2);
            cost_best = cost;
            best_mv_bits = mv_bits;
        }
    }

    /* search qpel-level motion vector*/
    if (pi->me_level > ME_LEV_HPEL && pi->curr_mvr == 0)
    {
        /* make MV to be absolute coordinate */
        cx = mv[MV_X] + ((s16)x << 2);
        cy = mv[MV_Y] + ((s16)y << 2);
        for (i = 0; i < pi->search_pattern_qpel_cnt; i++)
        {
            mv_x = cx + pi->search_pattern_qpel[i][0];
            mv_y = cy + pi->search_pattern_qpel[i][1];
            /* get MVD bits */
            mv_bits = get_mv_bits_with_mvr(mv_x - gmvp[MV_X], mv_y - gmvp[MV_Y], pi->num_refp, refi, pi->curr_mvr);
            mv_bits += bi ? 1 : 2; // add inter_dir bits
            if (bi)
            {
                mv_bits += pi->mot_bits[lidx_r];
            }
            /* get MVD cost_best */
            cost = MV_COST(pi, mv_bits);
            /* get the interpolated(predicted) image */
           
#if CU_LEVEL_PRIVACY
            com_mc_l(mv_x, mv_y, ref, mv_x, mv_y, s_ref, cu_stride, pred, w, h, bit_depth, privacy_map, map_s, map_h, cu_flag, pred_map);
#else
            com_mc_l(mv_x, mv_y, ref, mv_x, mv_y, s_ref, cu_stride, pred, w, h, bit_depth);
#endif

            if (bi)
            {
                /* get sad */
                cost += calc_satd_16b(w, h, org_bi, pred, cu_stride, cu_stride, bit_depth) >> 1;
            }
            else
            {
                /* get sad */
                cost += calc_satd_16b(w, h, org, pred, s_org, cu_stride, bit_depth);
            }
            /* check if motion cost_best is less than minimum cost_best */
            if (cost < cost_best)
            {
                mv[MV_X] = mv_x - ((s16)x << 2);
                mv[MV_Y] = mv_y - ((s16)y << 2);
                cost_best = cost;
                best_mv_bits = mv_bits;
            }
        }
    }
#if !ENC_ME_IMP
    if (!bi && best_mv_bits > 0)
    {
        pi->mot_bits[lidx] = best_mv_bits;
    }
#endif
    return cost_best;
}

```