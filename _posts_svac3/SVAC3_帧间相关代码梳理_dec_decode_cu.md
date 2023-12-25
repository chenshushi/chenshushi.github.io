---
layout:     post
title:      SVAC3_帧间相关代码梳理_dec_decode_cu函数
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
# dec_decode_cu
该函数是解码端的函数
```javascript
int dec_decode_cu(DEC_CTX * ctx, DEC_CORE * core
#if CU_LEVEL_PRIVACY
    , int privacy_flag
#endif
) // this function can be optimized to better match with the text
// 上面代码省略，主要集中看帧间部分
{    
    /* parse prediction info */
    /* parse prediction info */
    if (mod_info_curr->cu_mode == MODE_SKIP || mod_info_curr->cu_mode == MODE_DIR)
    {
        dec_derive_skip_direct_info(ctx, core);
        if (mod_info_curr->cu_mode == MODE_SKIP)
        {
            assert(mod_info_curr->pb_part == SIZE_2Nx2N);
        }
    }
#if USE_IBC
    else if (mod_info_curr->cu_mode == MODE_IBC)
    {
#if USE_SP
        if (mod_info_curr->ibc_flag)
        {
#endif
            decode_bvd(bs, sbac, mvd);
#if IBC_BVP
            s16 mvp[MV_D];
            dec_derive_ibc_bvp_info(ctx, core, mvp);
#if IBC_ABVR
            mvd[MV_X] = mvd[MV_X] << (mod_info_curr->bvr_idx + 2);
            mvd[MV_Y] = mvd[MV_Y] << (mod_info_curr->bvr_idx + 2);
            if (core->cnt_hbvp_cands >= 2)
            {
                com_mv_rounding_s16(mvp[MV_X], mvp[MV_Y], &mvp[MV_X], &mvp[MV_Y], mod_info_curr->bvr_idx + 2, mod_info_curr->bvr_idx + 2);
            }
#endif
            if (core->cnt_hbvp_cands >= 2)
            {
                mod_info_curr->mv[REFP_0][MV_X] = mvd[MV_X] + mvp[MV_X];
                mod_info_curr->mv[REFP_0][MV_Y] = mvd[MV_Y] + mvp[MV_Y];
            }
            else
            {
                mod_info_curr->mv[REFP_0][MV_X] = mvd[MV_X];
                mod_info_curr->mv[REFP_0][MV_Y] = mvd[MV_Y];
            }
            mod_info_curr->mv[REFP_0][MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mod_info_curr->mv[REFP_0][MV_X]);
            mod_info_curr->mv[REFP_0][MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mod_info_curr->mv[REFP_0][MV_Y]);
    
#else
#if IBC_ABVR
            mod_info_curr->mv[REFP_0][MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvd[MV_X] << (mod_info_curr->bvr_idx + 2));
            mod_info_curr->mv[REFP_0][MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvd[MV_Y] << (mod_info_curr->bvr_idx + 2));
#else
            mod_info_curr->mv[REFP_0][MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvd[MV_X] << 2);
            mod_info_curr->mv[REFP_0][MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvd[MV_Y] << 2);
#endif
#endif
            mod_info_curr->refi[REFP_0] = -1;
            mod_info_curr->refi[REFP_1] = -1;
            assert(dec_is_bv_valid(ctx, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height,
                        mod_info_curr->cu_width_log2, mod_info_curr->cu_height_log2,
                        ctx->info.pic_width, ctx->info.pic_height, mod_info_curr->mv[REFP_0][MV_X] >> 2, mod_info_curr->mv[REFP_0][MV_Y] >> 2,  ctx->info.sqh.log2_max_cu_width_height));
#if USE_SP
        }
        else if (mod_info_curr->sp_flag)
        {
            decode_sp_or_cs2_cu_flag(bs, sbac, cu_width, cu_height, mod_info_curr, ctx);
            if (mod_info_curr->cs2_flag == TRUE)
            {
                dec_eco_CS2(ctx, core, bs, sbac, mod_info_curr);
            }
            else
            {
                dec_eco_sp(ctx, core, bs, sbac, mod_info_curr, ctx->tree_status); 
            }
            mod_info_curr->refi[REFP_0] = -1;
            mod_info_curr->refi[REFP_1] = -1;
            mod_info_curr->mv[REFP_0][MV_X] = 0;
            mod_info_curr->mv[REFP_0][MV_Y] = 0;
        }
#endif
    }
#endif
    else if (mod_info_curr->cu_mode == MODE_INTER)
    {
        if (ctx->info.pic_header.slice_type == SLICE_P)
        {
            inter_dir = PRED_L0;
        }
        else /* if(ctx->info.pic_header.slice_type == SLICE_B) */
        {
            inter_dir = decode_inter_dir(bs, sbac, mod_info_curr->pb_part, ctx);
        }

        if (mod_info_curr->affine_flag) // affine inter motion vector
        {
            dec_eco_affine_motion_derive(ctx, core, bs, sbac, inter_dir, cu_width, cu_height, affine_mvp, mvd);
        }
        else
        {
#if SMVD
            if (ctx->info.sqh.smvd_enable_flag && inter_dir == PRED_BI
                && (ctx->ptr - ctx->refp[0][REFP_0].ptr == ctx->refp[0][REFP_1].ptr - ctx->ptr)
                && !mod_info_curr->mvp_from_hmvp_flag
               )
            {
                mod_info_curr->smvd_flag = (u8)decode_smvd_flag(bs, sbac, ctx);
            }
            else
            {
                mod_info_curr->smvd_flag = 0;
            }
#endif

            /* forward */
            if (inter_dir == PRED_L0 || inter_dir == PRED_BI)
            {
                s16 mvp[MV_D];
#if SMVD
                if (mod_info_curr->smvd_flag == 1)
                {
                    mod_info_curr->refi[REFP_0] = 0;
                }
                else
                {
#endif
                    mod_info_curr->refi[REFP_0] = (s8)decode_refidx(bs, sbac, ctx->dpm.num_refp[REFP_0]);
#if SMVD
                }
#endif
                // 这里是帧间MVP推导过程
                com_derive_mvp(ctx->info, mod_info_curr, ctx->ptr, REFP_0, mod_info_curr->refi[REFP_0], core->cnt_hmvp_cands,
                    core->motion_cands, ctx->map, ctx->refp, mod_info_curr->mvr_idx, mvp);
                // 帧间MVD解码过程
                decode_mvd(bs, sbac, mvd);
                s32 mv_x = (s32)mvp[MV_X] + ((s32)mvd[MV_X] << mod_info_curr->mvr_idx);
                s32 mv_y = (s32)mvp[MV_Y] + ((s32)mvd[MV_Y] << mod_info_curr->mvr_idx);
                mod_info_curr->mv[REFP_0][MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_x);
                mod_info_curr->mv[REFP_0][MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_y);

            }
            else
            {
                mod_info_curr->refi[REFP_0] = REFI_INVALID;
                mod_info_curr->mv[REFP_0][MV_X] = 0;
                mod_info_curr->mv[REFP_0][MV_Y] = 0;
            }

            /* backward */
            if (inter_dir == PRED_L1 || inter_dir == PRED_BI)
            {
                s16 mvp[MV_D];
#if SMVD
                if (mod_info_curr->smvd_flag == 1)
                {
                    mod_info_curr->refi[REFP_1] = 0;
                }
                else
                {
#endif
                    mod_info_curr->refi[REFP_1] = (s8)decode_refidx(bs, sbac, ctx->dpm.num_refp[REFP_1]);
#if SMVD
                }
#endif
                com_derive_mvp(ctx->info, mod_info_curr, ctx->ptr, REFP_1, mod_info_curr->refi[REFP_1], core->cnt_hmvp_cands,
                    core->motion_cands, ctx->map, ctx->refp, mod_info_curr->mvr_idx, mvp);

#if SMVD
                if (mod_info_curr->smvd_flag == 1)
                {
                    mvd[MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, -mvd[MV_X]);
                    mvd[MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, -mvd[MV_Y]);
                }
                else
                {
#endif
                    decode_mvd(bs, sbac, mvd);
#if SMVD
                }
#endif
                s32 mv_x = (s32)mvp[MV_X] + ((s32)mvd[MV_X] << mod_info_curr->mvr_idx);
                s32 mv_y = (s32)mvp[MV_Y] + ((s32)mvd[MV_Y] << mod_info_curr->mvr_idx);

                mod_info_curr->mv[REFP_1][MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_x);
                mod_info_curr->mv[REFP_1][MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_y);
            }
            else
            {
                mod_info_curr->refi[REFP_1] = REFI_INVALID;
                mod_info_curr->mv[REFP_1][MV_X] = 0;
                mod_info_curr->mv[REFP_1][MV_Y] = 0;
            }
        }
#if BGC
        if (ctx->info.sqh.bgc_enable_flag && (ctx->info.pic_header.slice_type == SLICE_B) && REFI_IS_VALID(mod_info_curr->refi[REFP_0]) && REFI_IS_VALID(mod_info_curr->refi[REFP_1]) && (cu_width * cu_height >= 256))
        {
            mod_info_curr->bgc_flag = decode_bgc_flag(bs, sbac, ctx);
            if (mod_info_curr->bgc_flag)
            {
                mod_info_curr->bgc_idx = decode_bgc_idx(bs, sbac, ctx);
            }
        }
#endif
    }
```
# com_derive_mvp
该函数是解码端构造MVP的函数
```javascript
void com_derive_mvp(COM_INFO info, COM_MODE *mod_info_curr, int ptr, int ref_list, int ref_idx, int cnt_hmvp_cands, COM_MOTION *motion_cands, COM_MAP map, COM_REFP(*refp)[REFP_NUM], int mvr_idx, s16 mvp[MV_D])
{
    int scup = mod_info_curr->scup;
    int emvp_flag = mod_info_curr->mvp_from_hmvp_flag;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;

#if EXT_AMVR_HMVP
    if (!emvp_flag)
    {
#endif
        com_get_mvp_default(&info, mod_info_curr, refp, &map, ptr, ref_list, ref_idx, mvr_idx, mvp);
#if EXT_AMVR_HMVP
    }
    else
    {
        if (cnt_hmvp_cands == 0)
        {
            mvp[MV_X] = 0;
            mvp[MV_Y] = 0;
        }
        else if (cnt_hmvp_cands < mvr_idx + 1)
        {
            COM_MOTION motion = motion_cands[cnt_hmvp_cands - 1];
            com_get_mvp_hmvp(motion, ref_list, ptr, ref_idx, mvp, refp, mvr_idx);
        }
        else
        {
            COM_MOTION motion = motion_cands[cnt_hmvp_cands - 1 - mvr_idx];
            com_get_mvp_hmvp(motion, ref_list, ptr, ref_idx, mvp, refp, mvr_idx);
        }
    }
#endif
}
```

# com_get_mvp_default
该函数是解码端构造MVP的函数

```javascript
void com_get_mvp_default(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, int ptr_cur, int lidx, s8 cur_refi,
                         u8 amvr_idx, s16 mvp[MV_D])
{
    int scup = mod_info_curr->scup;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;

    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    int h_scu = info->pic_height_in_scu;
    COM_PIC_HEADER * sh = &info->pic_header;

    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;
    s8(*map_refi)[REFP_NUM] = pic_map->map_refi;
    u32* map_scu = pic_map->map_scu;


    int cnt, hv, ptr_cur_ref;
    int mvPredType = MVPRED_xy_MIN;
    int rFrameL, rFrameU, rFrameUR;
    int neb_addr[NUM_AVS2_SPATIAL_MV], valid_flag[NUM_AVS2_SPATIAL_MV];
    s8 refi[NUM_AVS2_SPATIAL_MV];
    s16 MVPs[NUM_AVS2_SPATIAL_MV][MV_D];
#if CU_LEVEL_PRIVACY
    u8 cur_cu_level = mod_info_curr->cu_level;
#endif
    check_mvp_motion_availability(info, mod_info_curr, pic_map, neb_addr, valid_flag, lidx);
    ptr_cur_ref = refp[cur_refi][lidx].ptr;
    for (cnt = 0; cnt < NUM_AVS2_SPATIAL_MV; cnt++)
    {
#if CU_LEVEL_PRIVACY 
        if (valid_flag[cnt] && (refp[cur_refi][lidx].pic_privacy==NULL || REFI_P_IS_VALID(refp[cur_refi][lidx].pic_privacy[neb_addr[cnt]], cur_cu_level)))
#else
        if (valid_flag[cnt])
#endif
        {
            refi[cnt] = map_refi[neb_addr[cnt]][lidx];
            assert(REFI_IS_VALID(refi[cnt]));
            {
                int ptr_neb_ref = refp[refi[cnt]][lidx].ptr;
                scaling_mv1(ptr_cur, ptr_cur_ref, ptr_cur, ptr_neb_ref, map_mv[neb_addr[cnt]][lidx], MVPs[cnt]);
            }
        }
        else
        {
            refi[cnt] = REFI_INVALID;
            MVPs[cnt][MV_X] = 0;
            MVPs[cnt][MV_Y] = 0;
        }
    }
    rFrameL = refi[0];
    rFrameU = refi[1];
    rFrameUR = refi[2];
    if ((rFrameL != REFI_INVALID) && (rFrameU == REFI_INVALID) && (rFrameUR == REFI_INVALID))
    {
        mvPredType = MVPRED_L;
    }
    else if ((rFrameL == REFI_INVALID) && (rFrameU != REFI_INVALID) && (rFrameUR == REFI_INVALID))
    {
        mvPredType = MVPRED_U;
    }
    else if ((rFrameL == REFI_INVALID) && (rFrameU == REFI_INVALID) && (rFrameUR != REFI_INVALID))
    {
        mvPredType = MVPRED_UR;
    }

    for (hv = 0; hv < MV_D; hv++)
    {
        s32 mva = (s32)MVPs[0][hv], mvb = (s32)MVPs[1][hv], mvc = (s32)MVPs[2][hv];
        switch (mvPredType)
        {
        case MVPRED_xy_MIN:
            if ((mva < 0 && mvb > 0 && mvc > 0) || (mva > 0 && mvb < 0 && mvc < 0))
            {
                mvp[hv] = (s16)((mvb + mvc) / 2);
            }
            else if ((mvb < 0 && mva > 0 && mvc > 0) || (mvb > 0 && mva < 0 && mvc < 0))
            {
                mvp[hv] = (s16)((mvc + mva) / 2);
            }
            else if ((mvc < 0 && mva > 0 && mvb > 0) || (mvc > 0 && mva < 0 && mvb < 0))
            {
                mvp[hv] = (s16)((mva + mvb) / 2);
            }
            else
            {
                s32 mva_ext = abs(mva - mvb);
                s32 mvb_ext = abs(mvb - mvc);
                s32 mvc_ext = abs(mvc - mva);
                s32 pred_vec = min(mva_ext, min(mvb_ext, mvc_ext));
                if (pred_vec == mva_ext)
                {
                    mvp[hv] = (s16)((mva + mvb) / 2);
                }
                else if (pred_vec == mvb_ext)
                {
                    mvp[hv] = (s16)((mvb + mvc) / 2);
                }
                else
                {
                    mvp[hv] = (s16)((mvc + mva) / 2);
                }
            }
            break;
        case MVPRED_L:
            mvp[hv] = (s16)mva;
            break;
        case MVPRED_U:
            mvp[hv] = (s16)mvb;
            break;
        case MVPRED_UR:
            mvp[hv] = (s16)mvc;
            break;
        default:
            assert(0);
            break;
        }
    }

    // clip MVP after rounding (rounding process might result in 32768)
    int mvp_x, mvp_y;
    com_mv_rounding_s32((s32)mvp[MV_X], (s32)mvp[MV_Y], &mvp_x, &mvp_y, amvr_idx, amvr_idx);
    mvp[MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvp_x);
    mvp[MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvp_y);
}
```