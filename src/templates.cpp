/*
 * Author - Erez Raviv <erezraviv@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ============================================================
 * Templates file
 *
 * eccpm
 * crow
 * throttle cut
 * flaperon
 * elevon
 * v-tail
 * throttle hold
 * Aileron Differential
 * Spoilers
 * Snap Roll
 * ELE->Flap
 * Flap->ELE
 *
 *
 *
 * =============================================================
 * Assumptions:
 * All primary channels are per modi12x3
 * Each template added to the end of each channel
 *
 *
 *
 */

#include <stdint.h>
#include <string.h>
#include "ersky9x.h"
#include "templates.h"
#include "myeeprom.h"

static const char string_1[] = "Simple 4-CH";
static const char string_2[] = "T-Cut";
static const char string_3[] = "Sticky T-Cut";
static const char string_4[] = "V-Tail";
static const char string_5[] = "Elevon\\Delta";
static const char string_6[] = "Heli Setup";
static const char string_7[] = "Gyro Setup";
static const char string_8[] = "Servo Test";
static const char string_9[] = "Range Test";

const char *n_Templates[NUM_TEMPLATES] = {
    string_1,
    string_2,
    string_3,
    string_4,
    string_5,
    string_6,
    string_7,
    string_8,
    string_9
};

SKYMixData* setDest(uint8_t dch)
{
    uint8_t i = 0;
    while ((g_model.mixData[i].destCh<=dch) && (g_model.mixData[i].destCh) && (i<MAX_SKYMIXERS)) i++;
    if(i==MAX_SKYMIXERS) return &g_model.mixData[0];

    memmove(&g_model.mixData[i+1],&g_model.mixData[i],
            (MAX_SKYMIXERS-(i+1))*sizeof(MixData) );
    memset(&g_model.mixData[i],0,sizeof(MixData));
    g_model.mixData[i].destCh = dch;
    return &g_model.mixData[i];
}

void clearMixes()
{
    memset(g_model.mixData,0,sizeof(g_model.mixData)); //clear all mixes
}

void clearCurves()
{
    memset(g_model.curves5,0,sizeof(g_model.curves5)); //clear all curves
    memset(g_model.curves9,0,sizeof(g_model.curves9)); //clear all curves
}

void setCurve(uint8_t c, int8_t ar[])
{
    if(c<MAX_CURVE5) //5 pt curve
        for(uint8_t i=0; i<5; i++) g_model.curves5[c][i] = ar[i];
    else  //9 pt curve
        for(uint8_t i=0; i<9; i++) g_model.curves9[c-MAX_CURVE5][i] = ar[i];
}

void setSwitch(uint8_t idx, uint8_t func, int8_t v1, int8_t v2)
{
    g_model.customSw[idx-1].func = func;
    g_model.customSw[idx-1].v1   = v1;
    g_model.customSw[idx-1].v2   = v2;
    g_model.customSw[idx-1].andsw = 0;
}

uint8_t convert_mode_helper(uint8_t x)
{
    return modn12x3[ g_eeGeneral.stickMode*4 + (x) - 1] ;
}

void applyTemplate(uint8_t idx)
{
    int8_t heli_ar1[] = {-100, -20, 30, 70, 90};
    int8_t heli_ar2[] = {80, 70, 60, 70, 100};
    int8_t heli_ar3[] = {100, 90, 80, 90, 100};
    int8_t heli_ar4[] = {-30,  -15, 0, 50, 100};
    int8_t heli_ar5[] = {-100, -50, 0, 50, 100};

    SKYMixData *md = &g_model.mixData[0];

    //CC(STK)   -> vSTK
    //ICC(vSTK) -> STK
#define ICC(x) icc[(x)-1]
    uint8_t icc[4] = {0};
    for(uint8_t i=1; i<=4; i++) //generate inverse array
        for(uint8_t j=1; j<=4; j++) if(CC(i)==j) icc[j-1]=i;


    uint8_t j = 0;

    //Simple 4-Ch
    if(idx==j++) 
    {
        clearMixes();
        md=setDest(ICC(STK_RUD));  md->srcRaw=CM(STK_RUD);  md->weight=100;
        md=setDest(ICC(STK_ELE));  md->srcRaw=CM(STK_ELE);  md->weight=100;
        md=setDest(ICC(STK_THR));  md->srcRaw=CM(STK_THR);  md->weight=100;
        md=setDest(ICC(STK_AIL));  md->srcRaw=CM(STK_AIL);  md->weight=100;
    }

    //T-Cut
    if(idx==j++)
    {
        md=setDest(ICC(STK_THR));  md->srcRaw=MIX_MAX;  md->weight=-100;  md->swtch=DSW_THR;  md->mltpx=MLTPX_REP;
    }

    //sticky t-cut
    if(idx==j++)
    {
        md=setDest(ICC(STK_THR));  md->srcRaw=MIX_MAX;  md->weight=-100;  md->swtch=DSW_SWC;  md->mltpx=MLTPX_REP;
        md=setDest(14);            md->srcRaw=CH(14);   md->weight= 100;
        md=setDest(14);            md->srcRaw=MIX_MAX;  md->weight=-100;  md->swtch=DSW_SWB;  md->mltpx=MLTPX_REP;
        md=setDest(14);            md->srcRaw=MIX_MAX;  md->weight= 100;  md->swtch=DSW_THR;  md->mltpx=MLTPX_REP;

        setSwitch(0xB,CS_VNEG, CM(STK_THR), -99);
        setSwitch(0xC,CS_VPOS, CH(14), 0);
    }

    //V-Tail
    if(idx==j++) 
    {
        clearMixes();
        md=setDest(ICC(STK_RUD));  md->srcRaw=CM(STK_RUD);  md->weight= 100;
        md=setDest(ICC(STK_RUD));  md->srcRaw=CM(STK_ELE);  md->weight=-100;
        md=setDest(ICC(STK_ELE));  md->srcRaw=CM(STK_RUD);  md->weight= 100;
        md=setDest(ICC(STK_ELE));  md->srcRaw=CM(STK_ELE);  md->weight= 100;
    }

    //Elevon\\Delta
    if(idx==j++)
    {
        clearMixes();
        md=setDest(ICC(STK_ELE));  md->srcRaw=CM(STK_ELE);  md->weight= 100;
        md=setDest(ICC(STK_ELE));  md->srcRaw=CM(STK_AIL);  md->weight= 100;
        md=setDest(ICC(STK_AIL));  md->srcRaw=CM(STK_ELE);  md->weight= 100;
        md=setDest(ICC(STK_AIL));  md->srcRaw=CM(STK_AIL);  md->weight=-100;
    }


    //Heli Setup
    if(idx==j++)
    {
        clearMixes();  //This time we want a clean slate
        clearCurves();

        //Set up Mixes
        //3 cyclic channels
        md=setDest(1);  md->srcRaw=MIX_CYC1;  md->weight= 100;
        md=setDest(2);  md->srcRaw=MIX_CYC2;  md->weight= 100;
        md=setDest(3);  md->srcRaw=MIX_CYC3;  md->weight= 100;

        //rudder
        md=setDest(4);  md->srcRaw=CM(STK_RUD); md->weight=100;

        //Throttle
        md=setDest(5);  md->srcRaw=CM(STK_THR);  md->weight= 100; md->swtch= DSW_ID0; md->curve=CV(1); md->carryTrim=TRIM_OFF;
        md=setDest(5);  md->srcRaw=CM(STK_THR);  md->weight= 100; md->swtch= DSW_ID1; md->curve=CV(2); md->carryTrim=TRIM_OFF;
        md=setDest(5);  md->srcRaw=CM(STK_THR);  md->weight= 100; md->swtch= DSW_ID2; md->curve=CV(3); md->carryTrim=TRIM_OFF;
        md=setDest(5);  md->srcRaw=MIX_MAX;      md->weight=-100; md->swtch= DSW_THR; md->mltpx=MLTPX_REP;

        //gyro gain
        md=setDest(6);  md->srcRaw=MIX_FULL; md->weight=30; md->swtch=-DSW_GEA;

        //collective
        md=setDest(11); md->srcRaw=CM(STK_THR);  md->weight=70; md->swtch= DSW_ID0; md->curve=CV(4); md->carryTrim=TRIM_OFF;
        md=setDest(11); md->srcRaw=CM(STK_THR);  md->weight=70; md->swtch= DSW_ID1; md->curve=CV(5); md->carryTrim=TRIM_OFF;
        md=setDest(11); md->srcRaw=CM(STK_THR);  md->weight=70; md->swtch= DSW_ID2; md->curve=CV(6); md->carryTrim=TRIM_OFF;

        g_model.swashType = SWASH_TYPE_120;
        g_model.swashCollectiveSource = CH(11);

        //Set up Curves
        setCurve(CURVE5(1),heli_ar1);
        setCurve(CURVE5(2),heli_ar2);
        setCurve(CURVE5(3),heli_ar3);
        setCurve(CURVE5(4),heli_ar4);
        setCurve(CURVE5(5),heli_ar5);
        setCurve(CURVE5(6),heli_ar5);
    }

    //Gyro Gain
    if(idx==j++)
    {
        md=setDest(6);  md->srcRaw=STK_P2; md->weight= 50; md->swtch=-DSW_GEA; md->sOffset=100;
        md=setDest(6);  md->srcRaw=STK_P2; md->weight=-50; md->swtch= DSW_GEA; md->sOffset=100;
    }

    //Servo Test
    if(idx==j++)
    {
        md=setDest(15); md->srcRaw=CH(16);   md->weight= 100; md->speedUp = 80; md->speedDown = 80;
        md=setDest(16); md->srcRaw=MIX_FULL; md->weight= 110; md->swtch=DSW_SW1;
        md=setDest(16); md->srcRaw=MIX_MAX;  md->weight=-110; md->swtch=DSW_SW2; md->mltpx=MLTPX_REP;
        md=setDest(16); md->srcRaw=MIX_MAX;  md->weight= 110; md->swtch=DSW_SW3; md->mltpx=MLTPX_REP;

        setSwitch(1,CS_LESS,CH(15),CH(16));
        setSwitch(2,CS_VPOS,CH(15),   105);
        setSwitch(3,CS_VNEG,CH(15),  -105);
    }

    // Range Test
    if(idx==j++)
    {
        md=setDest(24); md->srcRaw=MIX_FULL; md->weight= 100; md->swtch=DSW_SW1; md->speedUp = 40; md->speedDown = 40;
        setSwitch(1,CS_TIME,4,4);
    }

    STORE_MODELVARS;
//    eeWaitComplete() ;

}


