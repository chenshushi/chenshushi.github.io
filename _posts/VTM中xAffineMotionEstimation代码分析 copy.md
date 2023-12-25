---
layout:     post
title:      VTM中xAffineMotionEstimation代码分析
subtitle:   v1.0版
date:       2023-08-16
author:     Shushi Chen
header-img: img/post-bg-cook.jpg
catalog: true
tags:
    - VTNM阅读笔记
---

![]({{site.baseurl}}/img/logo.png)

#### 发布信息

2023年8月10日。V0.0版。


# 函数作用

本函数用于搜索仿射的运动矢量，代码及注释如下：
```javascript
void InterSearch::xAffineMotionEstimation(CodingUnit& cu,
  CPelUnitBuf&    origBuf,
  VVENC_RefPicList      refPicList,
  Mv              acMvPred[3],
  int             iRefIdxPred,
  Mv              acMv[3],
  uint32_t&       ruiBits,
  VVENC_Distortion&     ruiCost,
  int&            mvpIdx,
  const AffineAMVPInfo& aamvpi,
  bool            bBi)
{
  if( cu.cs->sps->BCW && cu.BcwIdx != BCW_DEFAULT && !bBi && xReadBufferedAffineUniMv( cu, refPicList, iRefIdxPred, acMvPred, acMv, ruiBits, ruiCost, mvpIdx, aamvpi ) )
  {
    return;
  }

  int bestMvpIdx = mvpIdx;
  const int width = cu.Y().width;
  const int height = cu.Y().height;

  const Picture* refPic = cu.slice->getRefPic(refPicList, iRefIdxPred);

  // Set Origin YUV: pcYuv
  CPelUnitBuf*   pBuf = &origBuf;
  double        fWeight = 1.0;

  CPelUnitBuf  origBufTmpCnst;
  enum VVENC_DFunc distFunc = (cu.cs->slice->disableSATDForRd) ? VVENC_DF_SAD : DF_HAD;

  // if Bi, set to ( 2 * Org - ListX )
  if (bBi)
  {
    PelUnitBuf  origBufTmp = m_tmpStorageLCU.getCompactBuf(cu);
    // NOTE: Other buf contains predicted signal from another direction
    PelUnitBuf otherBuf = m_tmpPredStorage[1 - (int)refPicList].getCompactBuf( cu );
    origBufTmp.copyFrom(origBuf);
    origBufTmp.removeHighFreq(otherBuf, m_pcEncCfg->m_bClipForBiPredMeEnabled, cu.slice->clpRngs);

    origBufTmpCnst = m_tmpStorageLCU.getCompactBuf(cu);
    pBuf           = &origBufTmpCnst;
    fWeight        = xGetMEDistortionWeight(cu.BcwIdx, refPicList);
  }

  // pred YUV
  PelUnitBuf  predBuf = m_tmpAffiStorage.getCompactBuf(cu);

  // Set start Mv position, use input mv as started search mv
  Mv acMvTemp[3];
  ::memcpy(acMvTemp, acMv, sizeof(Mv) * 3);
  // Set delta mv
  // malloc buffer
  int iParaNum = cu.affineType ? 7 : 5;
  int affineParaNum = iParaNum - 1;
  int mvNum = cu.affineType ? 3 : 2;
  double **pdEqualCoeff;
  pdEqualCoeff = new double *[iParaNum];
  for (int i = 0; i < iParaNum; i++)
  {
    pdEqualCoeff[i] = new double[iParaNum];
  }

  int64_t  i64EqualCoeff[7][7];
  Pel    *piError = m_tmpAffiError;
  int    *pdDerivate[2];
  pdDerivate[0] = m_tmpAffiDeri[0];
  pdDerivate[1] = m_tmpAffiDeri[1];

  VVENC_Distortion uiCostBest = MAX_DISTORTION;
  uint32_t uiBitsBest = 0;

  // do motion compensation with origin mv

  clipMv(acMvTemp[0], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
  clipMv(acMvTemp[1], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
  if (cu.affineType == AFFINEMODEL_6PARAM)
  {
    clipMv(acMvTemp[2], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
  }

  acMvTemp[0].roundAffinePrecInternal2Amvr(cu.imv);
  acMvTemp[1].roundAffinePrecInternal2Amvr(cu.imv);
  if (cu.affineType == AFFINEMODEL_6PARAM)
  {
    acMvTemp[2].roundAffinePrecInternal2Amvr(cu.imv);
  }
  xPredAffineBlk(COMP_Y, cu, refPic, acMvTemp, predBuf, false, cu.cs->slice->clpRngs[COMP_Y], refPicList);

  // get error
  uiCostBest = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->bitDepths[VVENC_CH_L], COMP_Y, distFunc);

  // get cost with mv
  m_pcRdCost->setCostScale(0);
  uiBitsBest = ruiBits;
  DTRACE(g_trace_ctx, D_COMMON, " (%d) xx uiBitsBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiBitsBest);
  uiBitsBest += xCalcAffineMVBits(cu, acMvTemp, acMvPred);
  DTRACE(g_trace_ctx, D_COMMON, " (%d) yy uiBitsBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiBitsBest);
  uiCostBest = (VVENC_Distortion)(floor(fWeight * (double)uiCostBest) + (double)m_pcRdCost->getCost(uiBitsBest));

  DTRACE(g_trace_ctx, D_COMMON, " (%d) uiBitsBest=%d, uiCostBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiBitsBest, uiCostBest);

  ::memcpy(acMv, acMvTemp, sizeof(Mv) * 3);

  const int bufStride = pBuf->Y().stride;
  const int predBufStride = predBuf.Y().stride;
  Mv prevIterMv[7][3];
  int iIterTime;
  if (cu.affineType == AFFINEMODEL_6PARAM)
  {
    iIterTime = bBi ? 3 : 4;
  }
  else
  {
    iIterTime = bBi ? 3 : 5;
  }

  if (!cu.cs->sps->AffineType)// getUseAffineType())
  {
    iIterTime = bBi ? 5 : 7;
  }

  for (int iter = 0; iter<iIterTime; iter++)    // iterate loop
  {
    memcpy(prevIterMv[iter], acMvTemp, sizeof(Mv) * 3);
    /*********************************************************************************
    *                         use gradient to update mv
    *********************************************************************************/
    // get Error Matrix
    const Pel* pOrg = pBuf->Y().buf;
    Pel* pPred = predBuf.Y().buf;
    for (int j = 0; j< height; j++)
    {
      for (int i = 0; i< width; i++)
      {
        piError[i + j * width] = pOrg[i] - pPred[i];
      }
      pOrg += bufStride;
      pPred += predBufStride;
    }

    // sobel x direction
    // -1 0 1
    // -2 0 2
    // -1 0 1
    pPred = predBuf.Y().buf;
    m_HorizontalSobelFilter(pPred, predBufStride, pdDerivate[0], width, width, height);

    // sobel y direction
    // -1 -2 -1
    //  0  0  0
    //  1  2  1
    m_VerticalSobelFilter(pPred, predBufStride, pdDerivate[1], width, width, height);

    // solve delta x and y
    for (int row = 0; row < iParaNum; row++)
    {
      memset(&i64EqualCoeff[row][0], 0, iParaNum * sizeof(int64_t));
    }

    m_EqualCoeffComputer(piError, width, pdDerivate, width, i64EqualCoeff, width, height
      , (cu.affineType == AFFINEMODEL_6PARAM)
    );

    for (int row = 0; row < iParaNum; row++)
    {
      for (int i = 0; i < iParaNum; i++)
      {
        pdEqualCoeff[row][i] = (double)i64EqualCoeff[row][i];
      }
    }

    double dAffinePara[6];
    double dDeltaMv[6];
    Mv acDeltaMv[3];

    solveEqual(pdEqualCoeff, affineParaNum, dAffinePara);

    // convert to delta mv
    dDeltaMv[0] = dAffinePara[0];
    dDeltaMv[2] = dAffinePara[2];
    const bool extParams = cu.affineType == AFFINEMODEL_6PARAM;
    if (extParams)
    {
      dDeltaMv[1] = dAffinePara[1] * width + dAffinePara[0];
      dDeltaMv[3] = dAffinePara[3] * width + dAffinePara[2];
      dDeltaMv[4] = dAffinePara[4] * height + dAffinePara[0];
      dDeltaMv[5] = dAffinePara[5] * height + dAffinePara[2];
    }
    else
    {
      dDeltaMv[1] = dAffinePara[1] * width + dAffinePara[0];
      dDeltaMv[3] = -dAffinePara[3] * width + dAffinePara[2];
    }

    const int normShiftTab[3] = { MV_PRECISION_QUARTER - MV_PRECISION_INT, MV_PRECISION_SIXTEENTH - MV_PRECISION_INT, MV_PRECISION_QUARTER - MV_PRECISION_INT };
    const int stepShiftTab[3] = { MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL - MV_PRECISION_SIXTEENTH, MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER };
    const int multiShift = 1 << normShiftTab[cu.imv];
    const int mvShift = stepShiftTab[cu.imv];

    acDeltaMv[0] = Mv((int)(dDeltaMv[0] * multiShift + SIGN(dDeltaMv[0]) * 0.5) << mvShift, (int)(dDeltaMv[2] * multiShift + SIGN(dDeltaMv[2]) * 0.5) << mvShift);
    acDeltaMv[1] = Mv((int)(dDeltaMv[1] * multiShift + SIGN(dDeltaMv[1]) * 0.5) << mvShift, (int)(dDeltaMv[3] * multiShift + SIGN(dDeltaMv[3]) * 0.5) << mvShift);
    if (extParams)
    {
      acDeltaMv[2] = Mv((int)(dDeltaMv[4] * multiShift + SIGN(dDeltaMv[4]) * 0.5) << mvShift, (int)(dDeltaMv[5] * multiShift + SIGN(dDeltaMv[5]) * 0.5) << mvShift);
    }
    bool bAllZero = false;
    for (int i = 0; i < mvNum; i++)
    {
      Mv deltaMv = acDeltaMv[i];
      if (cu.imv == 2)
      {
        deltaMv.roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_HALF);
      }
      if (deltaMv.hor != 0 || deltaMv.ver != 0)
      {
        bAllZero = false;
        break;
      }
      bAllZero = true;
    }

    if (bAllZero)
      break;

    // do motion compensation with updated mv
    for (int i = 0; i < mvNum; i++)
    {
      acMvTemp[i] += acDeltaMv[i];
      acMvTemp[i].hor = VVENC_Clip3(MV_MIN, MV_MAX, acMvTemp[i].hor);
      acMvTemp[i].ver = VVENC_Clip3(MV_MIN, MV_MAX, acMvTemp[i].ver);
      acMvTemp[i].roundAffinePrecInternal2Amvr(cu.imv);

      clipMv(acMvTemp[i], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
    }

    xPredAffineBlk(COMP_Y, cu, refPic, acMvTemp, predBuf, false, cu.slice->clpRngs[COMP_Y], refPicList);

    // get error
    VVENC_Distortion uiCostTemp = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->bitDepths[VVENC_CH_L], COMP_Y, distFunc);
    DTRACE(g_trace_ctx, D_COMMON, " (%d) uiCostTemp=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiCostTemp);

    // get cost with mv
    m_pcRdCost->setCostScale(0);
    uint32_t uiBitsTemp = ruiBits;
    uiBitsTemp += xCalcAffineMVBits(cu, acMvTemp, acMvPred);
    uiCostTemp = (VVENC_Distortion)(floor(fWeight * (double)uiCostTemp) + (double)m_pcRdCost->getCost(uiBitsTemp));

    // store best cost and mv
    if (uiCostTemp < uiCostBest)
    {
      uiCostBest = uiCostTemp;
      uiBitsBest = uiBitsTemp;
      memcpy(acMv, acMvTemp, sizeof(Mv) * 3);
      mvpIdx = bestMvpIdx;
    }
    else if(m_pcEncCfg->m_Affine > 1)
    {
      break;
    }
  }

  auto checkCPMVRdCost = [&](Mv ctrlPtMv[3])
  {
    xPredAffineBlk(COMP_Y, cu, refPic, ctrlPtMv, predBuf, false, cu.slice->clpRngs[COMP_Y], refPicList);
    // get error
    VVENC_Distortion costTemp = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->bitDepths[VVENC_CH_L], COMP_Y, distFunc);
    // get cost with mv
    m_pcRdCost->setCostScale(0);
    uint32_t bitsTemp = ruiBits;
    bitsTemp += xCalcAffineMVBits(cu, ctrlPtMv, acMvPred);
    costTemp = (VVENC_Distortion)(floor(fWeight * (double)costTemp) + (double)m_pcRdCost->getCost(bitsTemp));
    // store best cost and mv
    if (costTemp < uiCostBest)
    {
      uiCostBest = costTemp;
      uiBitsBest = bitsTemp;
      ::memcpy(acMv, ctrlPtMv, sizeof(Mv) * 3);
    }
  };

  const uint32_t mvShiftTable[3] = { MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL - MV_PRECISION_INTERNAL, MV_PRECISION_INTERNAL - MV_PRECISION_INT };
  const uint32_t mvShift = mvShiftTable[cu.imv];
  if (uiCostBest <= AFFINE_ME_LIST_MVP_TH*m_hevcCost)
  {
    Mv mvPredTmp[3] = { acMvPred[0], acMvPred[1], acMvPred[2] };
    Mv mvME[3];
    ::memcpy(mvME, acMv, sizeof(Mv) * 3);
    Mv dMv = mvME[0] - mvPredTmp[0];

    for (int j = 0; j < mvNum; j++)
    {
      if ((!j && mvME[j] != mvPredTmp[j]) || (j && mvME[j] != (mvPredTmp[j] + dMv)))
      {
        ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);
        acMvTemp[j] = mvPredTmp[j];

        if (j)
          acMvTemp[j] += dMv;

        checkCPMVRdCost(acMvTemp);
      }
    }

    //keep the rotation/zoom;
    if (mvME[0] != mvPredTmp[0])
    {
      ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);
      for (int i = 1; i < mvNum; i++)
      {
        acMvTemp[i] -= dMv;
      }
      acMvTemp[0] = mvPredTmp[0];

      checkCPMVRdCost(acMvTemp);
    }

    //keep the translation;
    if (cu.affineType == AFFINEMODEL_6PARAM && mvME[1] != (mvPredTmp[1] + dMv) && mvME[2] != (mvPredTmp[2] + dMv))
    {
      ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);

      acMvTemp[1] = mvPredTmp[1] + dMv;
      acMvTemp[2] = mvPredTmp[2] + dMv;

      checkCPMVRdCost(acMvTemp);
    }

    // 8 nearest neighbor search
    int testPos[8][2] = { { -1, 0 },{ 0, -1 },{ 0, 1 },{ 1, 0 },{ -1, -1 },{ -1, 1 },{ 1, 1 },{ 1, -1 } };
    const int maxSearchRound = 3;

    for (int rnd = 0; rnd < maxSearchRound; rnd++)
    {
      bool modelChange = false;
      //search the model parameters with finear granularity;
      for (int j = 0; j < mvNum; j++)
      {
        bool loopChange = false;
        for (int iter = 0; iter < 2; iter++)
        {
          if (iter == 1 && !loopChange)
          {
            break;
          }
          Mv centerMv[3];
          memcpy(centerMv, acMv, sizeof(Mv) * 3);
          memcpy(acMvTemp, acMv, sizeof(Mv) * 3);

          for (int i = ((iter == 0) ? 0 : 4); i < ((iter == 0) ? 4 : 8); i++)
          {
            acMvTemp[j].set(centerMv[j].hor + (testPos[i][0] << mvShift), centerMv[j].ver + (testPos[i][1] << mvShift));
            clipMv(acMvTemp[j], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
            xPredAffineBlk(COMP_Y, cu, refPic, acMvTemp, predBuf, false, cu.slice->clpRngs[COMP_Y], refPicList);

            VVENC_Distortion costTemp = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->bitDepths[VVENC_CH_L], COMP_Y, distFunc);
            uint32_t bitsTemp = ruiBits;
            bitsTemp += xCalcAffineMVBits(cu, acMvTemp, acMvPred);
            costTemp = (VVENC_Distortion)(floor(fWeight * (double)costTemp) + (double)m_pcRdCost->getCost(bitsTemp));

            if (costTemp < uiCostBest)
            {
              uiCostBest = costTemp;
              uiBitsBest = bitsTemp;
              ::memcpy(acMv, acMvTemp, sizeof(Mv) * 3);
              modelChange = true;
              loopChange = true;
            }
          }
        }
      }

      if (!modelChange)
      {
        break;
      }
    }
  }
  acMvPred[0] = aamvpi.mvCandLT[mvpIdx];
  acMvPred[1] = aamvpi.mvCandRT[mvpIdx];
  acMvPred[2] = aamvpi.mvCandLB[mvpIdx];

  // free buffer
  for (int i = 0; i<iParaNum; i++)
    delete[]pdEqualCoeff[i];
  delete[]pdEqualCoeff;

  ruiBits = uiBitsBest;
  ruiCost = uiCostBest;
  DTRACE(g_trace_ctx, D_COMMON, " (%d) uiBitsBest=%d, uiCostBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiBitsBest, uiCostBest);
}
```


