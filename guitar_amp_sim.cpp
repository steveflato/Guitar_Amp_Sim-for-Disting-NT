// Guitar Amp Sim - distingNT
// Stereo or mono input, stereo out
// Boost -> Gate -> Drive -> IR L/R -> Width -> L-R Delay -> Dry/Wet -> Out Level
// GUID: FLMP

#include <distingnt/api.h>
#include <distingnt/wav.h>
#include <cstddef>
#include <new>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const int kMaxIrLen    = 128;
static const int kMaxDelayLen = 2048;  // ~20ms at 96kHz
static const char* kIrFolder  = "CabIR";

struct DCBlock {
    float x1, y1;
    void init() { x1=y1=0.f; }
    inline float process(float x, float R) {
        float y = x - x1 + R*y1; x1=x; y1=y; return y;
    }
};

enum {
    kParamInputL      = 0,
    kParamInputR      = 1,   // second input (mono if same as L)
    kParamOutputL     = 2,
    kParamOutputLMode = 3,
    kParamOutputR     = 4,
    kParamOutputRMode = 5,
    kParamInputGain   = 6,
    kParamBoost       = 7,
    kParamGate        = 8,
    kParamDriveType   = 9,
    kParamDrive       = 10,
    kParamStereoDelay = 11,
    kParamWidth       = 12,
    kParamDryWet      = 13,
    kParamOutputLevel = 14,
    kParamIrSelectL   = 15,
    kParamIrSelectR   = 16,
};

struct _IrChannel {
    float irBuffer [kMaxIrLen];
    float delayLine[kMaxIrLen];
};

struct _GAS_DRAM {
    _IrChannel L;
    _IrChannel R;
    float delayBuf[kMaxDelayLen];
};

struct _Alg : public _NT_algorithm {
    _GAS_DRAM*     dram;
    _NT_wavRequest wavRequestL;
    _NT_wavRequest wavRequestR;
    int   irLenL,    irLenR;
    int   delayPosL, delayPosR;
    bool  irLoadedL,     irLoadedR;
    bool  awaitingLoadL, awaitingLoadR;
    bool  cardMounted,   folderScanned;
    int   irFolderIndex, irSampleCount;
    float irNormGainL,   irNormGainR;
    DCBlock dcBlockL, dcBlockR;
    float gateEnvL, gateEnvR;
    int   delayWritePos;
    float peakIn, peakOut;
    _NT_parameter params[17];
};

static const char* kDriveTypeNames[] = {
    "Soft Clip", "Hard Clip", "Fuzz", "Tape", NULL
};

static const _NT_parameter kDefaultParams[] = {
    NT_PARAMETER_AUDIO_INPUT("Input L", 1, 1)                  // 0
    NT_PARAMETER_AUDIO_INPUT("Input R", 1, 1)                  // 1  same default=mono
    NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE("Output L", 1, 13)     // 2+3
    NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE("Output R", 1, 14)     // 4+5
    { "Input Gain", -20, 40,  0, kNT_unitDb,      0, NULL },   // 6
    { "Boost",        0, 30,  0, kNT_unitDb,      0, NULL },   // 7
    { "Noise Gate",   0,100,  0, kNT_unitPercent,  0, NULL },  // 8
    { "Drive Type",   0,  3,  0, kNT_unitEnum,    0, kDriveTypeNames }, // 9
    { "Drive",        0,100,  0, kNT_unitPercent,  0, NULL },  // 10
    { "L-R Delay",    0,200,  0, kNT_unitNone,    0, NULL },   // 11  0.1ms steps
    { "Width",        0,100,100,kNT_unitPercent,  0, NULL },   // 12
    { "Dry/Wet",      0,100,100,kNT_unitPercent,  0, NULL },   // 13
    { "Out Level",  -20, 20,  0, kNT_unitDb,       0, NULL },  // 14
    { "IR Left",      0,  0,  0, kNT_unitNone,    0, NULL },   // 15
    { "IR Right",     0,  0,  0, kNT_unitNone,    0, NULL },   // 16
};

static const uint8_t kPageIO[]  = { kParamInputL, kParamInputR,
                                    kParamOutputL, kParamOutputLMode,
                                    kParamOutputR, kParamOutputRMode };
static const uint8_t kPageAmp[] = { kParamInputGain, kParamBoost,
                                    kParamGate,
                                    kParamDriveType, kParamDrive };
static const uint8_t kPageMix[] = { kParamStereoDelay, kParamWidth,
                                    kParamDryWet, kParamOutputLevel };
static const uint8_t kPageCab[] = { kParamIrSelectL, kParamIrSelectR };

static const _NT_parameterPage kPages[] = {
    { "I/O", 6, 0, {0,0}, kPageIO  },
    { "Amp", 5, 0, {0,0}, kPageAmp },
    { "Mix", 4, 0, {0,0}, kPageMix },
    { "Cab", 2, 0, {0,0}, kPageCab },
};
static const _NT_parameterPages kParamPages = { 4, kPages };

// ---------------------------------------------------------------------------
//  IR loading
// ---------------------------------------------------------------------------
static float calcNorm(const float* buf, int len) {
    float peak = 0.f;
    for (int i = 0; i < len; i++) { float a=fabsf(buf[i]); if(a>peak) peak=a; }
    float n = (peak > 1e-6f) ? 1.f/peak : 1.f;
    return (n > 10.f) ? 10.f : n;
}

static void irLoadCallbackL(void* data, bool ok) {
    _Alg* p = (_Alg*)data; p->awaitingLoadL = false;
    if (!ok || p->irLenL==0) { p->irLoadedL=false; return; }
    p->irNormGainL = calcNorm(p->dram->L.irBuffer, p->irLenL);
    memset(p->dram->L.delayLine, 0, kMaxIrLen*sizeof(float));
    p->delayPosL = 0; p->irLoadedL = true;
}

static void irLoadCallbackR(void* data, bool ok) {
    _Alg* p = (_Alg*)data; p->awaitingLoadR = false;
    if (!ok || p->irLenR==0) { p->irLoadedR=false; return; }
    p->irNormGainR = calcNorm(p->dram->R.irBuffer, p->irLenR);
    memset(p->dram->R.delayLine, 0, kMaxIrLen*sizeof(float));
    p->delayPosR = 0; p->irLoadedR = true;
}

static void loadIr(_Alg* p, int sel, bool isRight) {
    if (p->irFolderIndex<0 || p->irSampleCount<=0) return;
    if (sel<0 || sel>=p->irSampleCount) return;
    if (isRight && p->awaitingLoadR) return;
    if (!isRight && p->awaitingLoadL) return;
    _NT_wavInfo info;
    NT_getSampleFileInfo((uint32_t)p->irFolderIndex, (uint32_t)sel, info);
    int frames = (int)info.numFrames;
    if (frames>kMaxIrLen) frames=kMaxIrLen;
    if (frames<=0) return;
    _NT_wavRequest* req = isRight ? &p->wavRequestR : &p->wavRequestL;
    req->folder=p->irFolderIndex; req->sample=sel;
    req->dst = isRight ? p->dram->R.irBuffer : p->dram->L.irBuffer;
    req->numFrames=frames; req->startOffset=0;
    req->channels=kNT_WavMono; req->bits=kNT_WavBits32;
    req->progress=kNT_WavNoProgress;
    req->callback = isRight ? irLoadCallbackR : irLoadCallbackL;
    req->callbackData = p;
    if (isRight) { p->irLenR=frames; p->irLoadedR=false; p->awaitingLoadR=NT_readSampleFrames(*req); }
    else         { p->irLenL=frames; p->irLoadedL=false; p->awaitingLoadL=NT_readSampleFrames(*req); }
}

static void scanFolder(_Alg* p, int algIdx) {
    p->folderScanned = true;
    uint32_t n = NT_getNumSampleFolders();
    for (uint32_t i=0; i<n; i++) {
        _NT_wavFolderInfo fi; NT_getSampleFolderInfo(i, fi);
        if (fi.name && strcmp(fi.name, kIrFolder)==0) {
            p->irFolderIndex=(int)i; p->irSampleCount=(int)fi.numSampleFiles;
            if (p->irSampleCount>0 && algIdx>=0) {
                p->params[kParamIrSelectL].max=p->irSampleCount-1;
                p->params[kParamIrSelectL].unit=kNT_unitHasStrings;
                p->params[kParamIrSelectR].max=p->irSampleCount-1;
                p->params[kParamIrSelectR].unit=kNT_unitHasStrings;
                NT_updateParameterDefinition((uint32_t)algIdx, kParamIrSelectL);
                NT_updateParameterDefinition((uint32_t)algIdx, kParamIrSelectR);
            }
            loadIr(p, p->v[kParamIrSelectL], false);
            loadIr(p, p->v[kParamIrSelectR], true);
            return;
        }
    }
}

// ---------------------------------------------------------------------------
//  API
// ---------------------------------------------------------------------------
static void calculateRequirements(_NT_algorithmRequirements& req, const int32_t*) {
    req.numParameters = ARRAY_SIZE(kDefaultParams);
    req.sram = sizeof(_Alg);
    req.dram = sizeof(_GAS_DRAM);
    req.dtc = req.itc = 0;
}

static _NT_algorithm* construct(const _NT_algorithmMemoryPtrs& ptrs,
                                const _NT_algorithmRequirements&, const int32_t*) {
    _Alg* alg = new (ptrs.sram) _Alg();
    alg->dram = (_GAS_DRAM*)ptrs.dram;
    memset(alg->dram, 0, sizeof(_GAS_DRAM));
    memcpy(alg->params, kDefaultParams, sizeof(kDefaultParams));
    alg->parameters=alg->params; alg->parameterPages=&kParamPages;
    alg->irLenL=alg->irLenR=0;
    alg->delayPosL=alg->delayPosR=0;
    alg->irLoadedL=alg->irLoadedR=false;
    alg->awaitingLoadL=alg->awaitingLoadR=false;
    alg->cardMounted=alg->folderScanned=false;
    alg->irFolderIndex=-1; alg->irSampleCount=0;
    alg->irNormGainL=alg->irNormGainR=1.f;
    alg->dcBlockL.init(); alg->dcBlockR.init();
    alg->gateEnvL=alg->gateEnvR=0.f;
    alg->delayWritePos=0;
    alg->peakIn=alg->peakOut=0.f;
    return alg;
}

static void parameterChanged(_NT_algorithm* self, int p) {
    _Alg* pThis=(_Alg*)self;
    if (p==kParamIrSelectL) loadIr(pThis, pThis->v[kParamIrSelectL], false);
    if (p==kParamIrSelectR) loadIr(pThis, pThis->v[kParamIrSelectR], true);
}

static int parameterString(_NT_algorithm* self, int p, int v, char* buff) {
    _Alg* pThis=(_Alg*)self;
    if ((p==kParamIrSelectL||p==kParamIrSelectR) &&
        pThis->irFolderIndex>=0 && v>=0 && v<pThis->irSampleCount) {
        _NT_wavInfo info;
        NT_getSampleFileInfo((uint32_t)pThis->irFolderIndex,(uint32_t)v,info);
        if (info.name) {
            strncpy(buff,info.name,kNT_parameterStringSize-1);
            buff[kNT_parameterStringSize-1]=0;
            return (int)strlen(buff);
        }
    }
    return 0;
}

// ---------------------------------------------------------------------------
//  Saturation
// ---------------------------------------------------------------------------
static inline float saturate(float s, int type, float drive) {
    if (drive<=0.f) return s;
    float pregain=1.f+drive*7.f, x=s*pregain, sat;
    switch(type) {
        case 1: { float t=1.f-drive*0.6f; sat=(x>t)?t:(x<-t)?-t:x; } break;
        case 2: sat=tanhf(x*(1.f+fabsf(x)*0.5f)*1.5f); break;
        case 3: { float xc=x*0.8f; if(xc>1.f)xc=1.f; if(xc<-1.f)xc=-1.f;
                  sat=xc-(xc*xc*xc)/3.f+xc*xc*0.1f; } break;
        default: sat=tanhf(x); break;
    }
    return sat/pregain;
}

// ---------------------------------------------------------------------------
//  step
// ---------------------------------------------------------------------------
static void step(_NT_algorithm* self, float* busFrames, int numFramesBy4) {
    _Alg* pThis=(_Alg*)self;
    int numFrames=numFramesBy4*4;

    int inBusL  = pThis->v[kParamInputL]  - 1;
    int inBusR  = pThis->v[kParamInputR]  - 1;
    int outBusL = pThis->v[kParamOutputL] - 1;
    int outBusR = pThis->v[kParamOutputR] - 1;
    if (inBusL<0 || outBusL<0 || outBusR<0) return;
    // If Input R is unset or same as L, treat as mono input
    bool stereoIn = (inBusR>=0 && inBusR!=inBusL);

    bool mounted=NT_isSdCardMounted();
    if (mounted!=pThis->cardMounted) { pThis->cardMounted=mounted; pThis->folderScanned=false; }
    if (mounted && !pThis->folderScanned) scanFolder(pThis, NT_algorithmIndex(self));

    const float* inL  = busFrames + inBusL  * numFrames;
    const float* inR  = stereoIn ? busFrames + inBusR * numFrames : inL;
    float*       outL = busFrames + outBusL * numFrames;
    float*       outR = busFrames + outBusR * numFrames;
    bool replaceL=(pThis->v[kParamOutputLMode]!=0);
    bool replaceR=(pThis->v[kParamOutputRMode]!=0);

    float sr      = (float)NT_globals.sampleRate;
    float inGain  = powf(10.f, pThis->v[kParamInputGain]  / 20.f);
    float boost   = powf(10.f, pThis->v[kParamBoost]      / 20.f);
    float outGain = powf(10.f, pThis->v[kParamOutputLevel] / 20.f);
    float drive   = pThis->v[kParamDrive]  / 100.f;
    float width   = pThis->v[kParamWidth]  / 100.f;
    float wet     = pThis->v[kParamDryWet] / 100.f;
    float dry     = 1.f - wet;
    int   drvType = pThis->v[kParamDriveType];
    float dcR     = 1.f - 2.f*(float)M_PI*40.f/sr;

    bool  useGate = (pThis->v[kParamGate]>0);
    float gateThresh=0.f, gateAtk=0.f, gateRel=0.f;
    if (useGate) {
        float gateDb = -60.f + pThis->v[kParamGate]*54.f/100.f;
        gateThresh = powf(10.f, gateDb/20.f);
        gateAtk    = 1.f - expf(-1.f/(0.001f*sr));
        gateRel    = 1.f - expf(-1.f/(0.050f*sr));
    }

    int delayTenths  = pThis->v[kParamStereoDelay];
    if (delayTenths<0) delayTenths=0;
    int delaySamples = (int)((float)delayTenths * sr / 10000.f);
    if (delaySamples>=kMaxDelayLen) delaySamples=kMaxDelayLen-1;
    bool useDelay = (delaySamples>0);

    float* irL   = pThis->dram->L.irBuffer;
    float* dlL   = pThis->dram->L.delayLine;
    float* irR   = pThis->dram->R.irBuffer;
    float* dlR   = pThis->dram->R.delayLine;
    float* dlyBuf= pThis->dram->delayBuf;
    int lenL=pThis->irLenL, lenR=pThis->irLenR;
    float normL=pThis->irNormGainL, normR=pThis->irNormGainR;
    int dposL=pThis->delayPosL, dposR=pThis->delayPosR;
    int dwPos=pThis->delayWritePos;
    bool hasIrL=pThis->irLoadedL&&lenL>0;
    bool hasIrR=pThis->irLoadedR&&lenR>0;

    float gateEnvL=pThis->gateEnvL, gateEnvR=pThis->gateEnvR;
    float peakIn=pThis->peakIn*0.999f, peakOut=pThis->peakOut*0.999f;

    float tmpL[128], tmpR[128];
    int done=0;
    while (done<numFrames) {
        int batch=numFrames-done; if(batch>128) batch=128;
        for (int i=0;i<batch;i++) { tmpL[i]=inL[done+i]; tmpR[i]=inR[done+i]; }

        for (int i=0;i<batch;i++) {
            // DC block + input gain + boost
            float sL = pThis->dcBlockL.process(tmpL[i]*inGain, dcR) * boost;
            float sR = pThis->dcBlockR.process(tmpR[i]*inGain, dcR) * boost;

            float aL=fabsf(sL), aR=fabsf(sR);
            if (aL>peakIn) peakIn=aL;

            // Gate — independent per channel
            if (useGate) {
                aL>gateEnvL ? (gateEnvL+=gateAtk*(aL-gateEnvL))
                            : (gateEnvL+=gateRel*(aL-gateEnvL));
                aR>gateEnvR ? (gateEnvR+=gateAtk*(aR-gateEnvR))
                            : (gateEnvR+=gateRel*(aR-gateEnvR));
                if (gateEnvL<=gateThresh) sL=0.f;
                if (gateEnvR<=gateThresh) sR=0.f;
            }

            // Saturation
            sL = saturate(sL, drvType, drive);
            sR = saturate(sR, drvType, drive);

            // Left IR
            float wetL=sL;
            if (hasIrL) {
                dlL[dposL]=sL; wetL=0.f;
                int rpos=dposL;
                for (int k=0;k<lenL;k++) { wetL+=irL[k]*dlL[rpos]; if(--rpos<0) rpos=lenL-1; }
                wetL*=normL; if(++dposL>=lenL) dposL=0;
            }

            // Right IR
            float wetR=sR;
            if (hasIrR) {
                dlR[dposR]=sR; wetR=0.f;
                int rpos=dposR;
                for (int k=0;k<lenR;k++) { wetR+=irR[k]*dlR[rpos]; if(--rpos<0) rpos=lenR-1; }
                wetR*=normR; if(++dposR>=lenR) dposR=0;
            }

            // Stereo width
            float mid=(wetL+wetR)*0.5f;
            float finalL = (mid + (wetL-mid)*width) * outGain;
            float finalR = (mid + (wetR-mid)*width) * outGain;

            // Dry/wet
            float dryL = sL*outGain, dryR = sR*outGain;
            finalL = dry*dryL + wet*finalL;
            finalR = dry*dryR + wet*finalR;

            // L-R delay (Haas widening)
            if (useDelay) {
                dlyBuf[dwPos]=finalL;
                int rpos=dwPos-delaySamples;
                if (rpos<0) rpos+=kMaxDelayLen;
                finalR=dlyBuf[rpos];
                if(++dwPos>=kMaxDelayLen) dwPos=0;
            }

            float absOut=fabsf(finalL); if(absOut>peakOut) peakOut=absOut;

            outL[done+i] = replaceL ? finalL : outL[done+i]+finalL;
            outR[done+i] = replaceR ? finalR : outR[done+i]+finalR;
        }
        done+=batch;
    }

    pThis->delayPosL=dposL; pThis->delayPosR=dposR;
    pThis->delayWritePos=dwPos;
    pThis->gateEnvL=gateEnvL; pThis->gateEnvR=gateEnvR;
    pThis->peakIn=peakIn; pThis->peakOut=peakOut;
}

// ---------------------------------------------------------------------------
//  draw
// ---------------------------------------------------------------------------
static void drawBar(int x,int y,int w,int h,int val,int col) {
    NT_drawShapeI(kNT_box,x,y,x+w,y+h,5);
    int fill=(val*(w-2))/100; if(fill>0) NT_drawShapeI(kNT_rectangle,x+1,y+1,x+1+fill,y+h-1,col);
}
static void drawMeter(int x,int y,int w,int h,float val) {
    NT_drawShapeI(kNT_box,x,y,x+w,y+h,5);
    int fill=(int)(val*(w-2)); if(fill>w-2) fill=w-2;
    if(fill>0) { int col=(fill<(w-2)*6/10)?10:(fill<(w-2)*9/10)?14:12;
                 NT_drawShapeI(kNT_rectangle,x+1,y+1,x+1+fill,y+h-1,col); }
}

static bool draw(_NT_algorithm* self) {
    _Alg* pThis=(_Alg*)self;

    // Title
    NT_drawShapeI(kNT_rectangle,0,0,255,10,10);
    NT_drawText(128,8,"Guitar Amp Sim",15,kNT_textCentre,kNT_textNormal);

    // Left: Amp params
    NT_drawText(2,19,"IN",8,kNT_textLeft,kNT_textTiny);
    int inVal=(int)((pThis->v[kParamInputGain]+20)*100/60);
    if(inVal<0)inVal=0; if(inVal>100)inVal=100;
    drawBar(14,12,58,6,inVal,12);

    NT_drawText(2,27,"BS",8,kNT_textLeft,kNT_textTiny);
    drawBar(14,20,58,6,(int)(pThis->v[kParamBoost]*100/30),13);

    NT_drawText(2,35,"GT",8,kNT_textLeft,kNT_textTiny);
    drawBar(14,28,58,6,pThis->v[kParamGate],pThis->v[kParamGate]>0?10:5);

    NT_drawText(2,43,"DR",8,kNT_textLeft,kNT_textTiny);
    drawBar(14,36,58,6,pThis->v[kParamDrive],11);

    static const char* dNames[]={"Soft","Hard","Fuzz","Tape"};
    int dt=pThis->v[kParamDriveType];
    NT_drawText(2,51,(dt>=0&&dt<=3)?dNames[dt]:"",9,kNT_textLeft,kNT_textTiny);

    // Stereo input indicator
    bool stereoIn=(pThis->v[kParamInputR]>0 &&
                   pThis->v[kParamInputR]!=pThis->v[kParamInputL]);
    NT_drawText(40,51,stereoIn?"ST":"MN",8,kNT_textLeft,kNT_textTiny);

    // Divider
    NT_drawShapeI(kNT_line,79,11,79,63,6);

    // Middle: meters + mix
    NT_drawText(83,19,"IN",7,kNT_textLeft,kNT_textTiny);
    drawMeter(83,20,36,6,pThis->peakIn);
    NT_drawText(83,27,"OUT",7,kNT_textLeft,kNT_textTiny);
    drawMeter(83,28,36,6,pThis->peakOut);

    NT_drawText(83,37,"WD",7,kNT_textLeft,kNT_textTiny);
    drawBar(95,30,24,6,pThis->v[kParamWidth],9);
    NT_drawText(83,45,"MX",7,kNT_textLeft,kNT_textTiny);
    drawBar(95,38,24,6,pThis->v[kParamDryWet],9);

    int lvVal=(int)((pThis->v[kParamOutputLevel]+20)*100/40);
    if(lvVal<0)lvVal=0; if(lvVal>100)lvVal=100;
    NT_drawText(83,53,"LV",7,kNT_textLeft,kNT_textTiny);
    drawBar(95,46,24,6,lvVal,12);

    // L-R delay
    char dbuf[16];
    NT_drawText(83,61,"DL",7,kNT_textLeft,kNT_textTiny);
    NT_floatToString(dbuf,(float)pThis->v[kParamStereoDelay]*0.1f,1);
    NT_drawText(95,61,dbuf,8,kNT_textLeft,kNT_textTiny);

    // Divider
    NT_drawShapeI(kNT_line,122,11,122,63,6);

    // Right: IR names
    NT_drawText(125,19,"CAB",8,kNT_textLeft,kNT_textTiny);
    if (!pThis->cardMounted) {
        NT_drawText(125,30,"No SD",7,kNT_textLeft,kNT_textTiny);
    } else if (pThis->irFolderIndex<0) {
        NT_drawText(125,30,"No CabIR",7,kNT_textLeft,kNT_textTiny);
    } else if (pThis->awaitingLoadL||pThis->awaitingLoadR) {
        NT_drawText(125,30,"Loading...",9,kNT_textLeft,kNT_textTiny);
    } else {
        auto showIr=[&](int param,int y,const char* prefix,bool loaded){
            NT_drawShapeI(kNT_rectangle,125,y-4,130,y+1,loaded?10:5);
            NT_drawText(133,y,prefix,8,kNT_textLeft,kNT_textTiny);
            int sel=pThis->v[param];
            if (sel>=0&&sel<pThis->irSampleCount) {
                _NT_wavInfo info;
                NT_getSampleFileInfo((uint32_t)pThis->irFolderIndex,(uint32_t)sel,info);
                if (info.name) NT_drawText(145,y,info.name,11,kNT_textLeft,kNT_textTiny);
            }
        };
        showIr(kParamIrSelectL,27,"L",pThis->irLoadedL);
        showIr(kParamIrSelectR,37,"R",pThis->irLoadedR);
        NT_drawText(125,50,"W:",7,kNT_textLeft,kNT_textTiny);
        drawBar(135,44,30,6,pThis->v[kParamWidth],9);
    }
    return true;
}

// ---------------------------------------------------------------------------
//  Factory
// ---------------------------------------------------------------------------
static const _NT_factory factory = {
    .guid = NT_MULTICHAR('F','L','M','P'),
    .name = "Guitar Amp Sim",
    .calculateRequirements = calculateRequirements,
    .construct = construct,
    .parameterChanged = parameterChanged,
    .step = step,
    .draw = draw,
    .tags = kNT_tagEffect,
    .parameterString = parameterString,
};

extern "C"
uintptr_t pluginEntry(_NT_selector selector, uint32_t data) {
    switch (selector) {
        case kNT_selector_version:      return kNT_apiVersionCurrent;
        case kNT_selector_numFactories: return 1;
        case kNT_selector_factoryInfo:  return (uintptr_t)((data==0)?&factory:NULL);
    }
    return 0;
}
