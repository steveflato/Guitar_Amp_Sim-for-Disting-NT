// Guitar Amp Sim - distingNT
// Mono in, stereo out
// Input Gain + Noise Gate + Drive + IR L/R + Stereo Width + Dry/Wet + Output Level
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

static const int   kMaxIrLen = 128;
static const char* kIrFolder = "CabIR";

// DC blocker — proven stable
struct DCBlock {
    float x1, y1;
    void init() { x1=y1=0.f; }
    inline float process(float x, float R) {
        float y = x - x1 + R*y1;
        x1=x; y1=y; return y;
    }
};

enum {
    kParamInput       = 0,
    kParamOutputL     = 1,
    kParamOutputLMode = 2,
    kParamOutputR     = 3,
    kParamOutputRMode = 4,
    kParamInputGain   = 5,
    kParamBoost       = 6,   // clean pre-drive boost
    kParamGate        = 7,
    kParamCompThresh  = 8,   // compressor threshold
    kParamCompRatio   = 9,   // compressor ratio
    kParamCompRelease = 10,  // compressor release
    kParamMakeup      = 11,  // compressor make-up gain
    kParamDriveType   = 12,
    kParamDrive       = 13,
    kParamWidth       = 14,
    kParamDryWet      = 15,
    kParamOutputLevel = 16,
    kParamIrSelectL   = 17,
    kParamIrSelectR   = 18,
    kParamLimiter     = 19,  // limiter ceiling dB (0=off)
};

struct _IrChannel {
    float irBuffer [kMaxIrLen];
    float delayLine[kMaxIrLen];
};

struct _GAS_DRAM {
    _IrChannel L;
    _IrChannel R;
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
    DCBlock dcBlock;
    // Noise gate envelope follower state
    float gateEnv;
    // Peak hold for display (updated in step, read in draw)
    float peakIn, peakOut;
    // Compressor state
    float compEnv;
    float debugGainRed;
    float limEnv;
    _NT_parameter params[20];
};

static const char* kDriveTypeNames[] = {
    "Soft Clip", "Hard Clip", "Fuzz", "Tape", NULL
};

static const _NT_parameter kDefaultParams[] = {
    NT_PARAMETER_AUDIO_INPUT("Input", 1, 1)                    // 0
    NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE("Output L", 1, 13)     // 1+2
    NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE("Output R", 1, 14)     // 3+4
    { "Input Gain", -20, 40,  0, kNT_unitDb,      0, NULL },   // 5
    { "Boost",         0, 30,  0, kNT_unitDb,      0, NULL },   // 6  clean boost dB
    { "Noise Gate",    0,100,  0, kNT_unitPercent,  0, NULL },   // 7
    { "Comp Thresh", -80,  0,-40, kNT_unitDb,      0, NULL },   // 8
    { "Comp Ratio",    1, 20,  4, kNT_unitNone,    0, NULL },   // 9  1=off, 20=limiting
    { "Comp Release",  10,500,100, kNT_unitMs,      0, NULL },   // 10
    { "Makeup Gain",   0, 40,  0, kNT_unitDb,      0, NULL },   // 11
    { "Drive Type",    0,  3,  0, kNT_unitEnum,    0, kDriveTypeNames }, // 12
    { "Drive",         0,100,  0, kNT_unitPercent,  0, NULL },   // 12
    { "Width",         0,100,100,kNT_unitPercent,  0, NULL },    // 13
    { "Dry/Wet",       0,100,100,kNT_unitPercent,  0, NULL },    // 14
    { "Out Level",   -20, 20,  0, kNT_unitDb,       0, NULL },   // 15
    { "IR Left",       0,  0,  0, kNT_unitNone,     0, NULL },   // 16
    { "IR Right",      0,  0,  0, kNT_unitNone,     0, NULL },   // 17
    { "Limiter",      -30,  0,  0, kNT_unitDb,        0, NULL },   // 19  0=off
};

static const uint8_t kPageIO[]  = { kParamInput,
                                    kParamOutputL, kParamOutputLMode,
                                    kParamOutputR, kParamOutputRMode };
static const uint8_t kPageAmp[] = { kParamInputGain, kParamBoost,
                                    kParamGate,
                                    kParamCompThresh, kParamCompRatio,
                                    kParamCompRelease, kParamMakeup,
                                    kParamDriveType, kParamDrive };
static const uint8_t kPageMix[] = { kParamWidth, kParamDryWet, kParamOutputLevel };
static const uint8_t kPageCab[] = { kParamIrSelectL, kParamIrSelectR };
static const uint8_t kPageLim[] = { kParamLimiter };

static const _NT_parameterPage kPages[] = {
    { "I/O", 5, 0, {0,0}, kPageIO  },
    { "Amp", 9, 0, {0,0}, kPageAmp },
    { "Mix", 3, 0, {0,0}, kPageMix },
    { "Cab", 2, 0, {0,0}, kPageCab },
    { "Lim", 1, 0, {0,0}, kPageLim },
};
static const _NT_parameterPages kParamPages = { 5, kPages };

// ---------------------------------------------------------------------------
//  IR loading
// ---------------------------------------------------------------------------
static float calcNorm(const float* buf, int len) {
    float peak = 0.f;
    for (int i = 0; i < len; i++) {
        float a = fabsf(buf[i]);
        if (a > peak) peak = a;
    }
    float n = (peak > 1e-6f) ? 1.f/peak : 1.f;
    return (n > 10.f) ? 10.f : n;
}

static void irLoadCallbackL(void* data, bool ok) {
    _Alg* p = (_Alg*)data;
    p->awaitingLoadL = false;
    if (!ok || p->irLenL == 0) { p->irLoadedL = false; return; }
    p->irNormGainL = calcNorm(p->dram->L.irBuffer, p->irLenL);
    memset(p->dram->L.delayLine, 0, kMaxIrLen * sizeof(float));
    p->delayPosL = 0;
    p->irLoadedL = true;
}

static void irLoadCallbackR(void* data, bool ok) {
    _Alg* p = (_Alg*)data;
    p->awaitingLoadR = false;
    if (!ok || p->irLenR == 0) { p->irLoadedR = false; return; }
    p->irNormGainR = calcNorm(p->dram->R.irBuffer, p->irLenR);
    memset(p->dram->R.delayLine, 0, kMaxIrLen * sizeof(float));
    p->delayPosR = 0;
    p->irLoadedR = true;
}

static void loadIr(_Alg* p, int sel, bool isRight) {
    if (p->irFolderIndex < 0 || p->irSampleCount <= 0) return;
    if (sel < 0 || sel >= p->irSampleCount) return;
    if (isRight && p->awaitingLoadR) return;
    if (!isRight && p->awaitingLoadL) return;
    _NT_wavInfo info;
    NT_getSampleFileInfo((uint32_t)p->irFolderIndex, (uint32_t)sel, info);
    int frames = (int)info.numFrames;
    if (frames > kMaxIrLen) frames = kMaxIrLen;
    if (frames <= 0) return;
    _NT_wavRequest* req = isRight ? &p->wavRequestR : &p->wavRequestL;
    req->folder       = (uint32_t)p->irFolderIndex;
    req->sample       = (uint32_t)sel;
    req->dst          = isRight ? p->dram->R.irBuffer : p->dram->L.irBuffer;
    req->numFrames    = (uint32_t)frames;
    req->startOffset  = 0;
    req->channels     = kNT_WavMono;
    req->bits         = kNT_WavBits32;
    req->progress     = kNT_WavNoProgress;
    req->callback     = isRight ? irLoadCallbackR : irLoadCallbackL;
    req->callbackData = p;
    if (isRight) {
        p->irLenR = frames; p->irLoadedR = false;
        p->awaitingLoadR = NT_readSampleFrames(*req);
    } else {
        p->irLenL = frames; p->irLoadedL = false;
        p->awaitingLoadL = NT_readSampleFrames(*req);
    }
}

static void scanFolder(_Alg* p, int algIdx) {
    p->folderScanned = true;
    uint32_t n = NT_getNumSampleFolders();
    for (uint32_t i = 0; i < n; i++) {
        _NT_wavFolderInfo fi;
        NT_getSampleFolderInfo(i, fi);
        if (fi.name && strcmp(fi.name, kIrFolder) == 0) {
            p->irFolderIndex = (int)i;
            p->irSampleCount = (int)fi.numSampleFiles;
            if (p->irSampleCount > 0 && algIdx >= 0) {
                p->params[kParamIrSelectL].max  = p->irSampleCount - 1;
                p->params[kParamIrSelectL].unit = kNT_unitHasStrings;
                p->params[kParamIrSelectR].max  = p->irSampleCount - 1;
                p->params[kParamIrSelectR].unit = kNT_unitHasStrings;
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
    alg->parameters     = alg->params;
    alg->parameterPages = &kParamPages;
    alg->irLenL = alg->irLenR = 0;
    alg->delayPosL = alg->delayPosR = 0;
    alg->irLoadedL = alg->irLoadedR = false;
    alg->awaitingLoadL = alg->awaitingLoadR = false;
    alg->cardMounted = alg->folderScanned = false;
    alg->irFolderIndex = -1;
    alg->irSampleCount = 0;
    alg->irNormGainL = alg->irNormGainR = 1.f;
    alg->dcBlock.init();
    alg->gateEnv = 0.f;
    alg->compEnv = 0.f;
    alg->debugGainRed = 1.f;
    alg->limEnv = 0.f;
    alg->peakIn = alg->peakOut = 0.f;
    return alg;
}

static void parameterChanged(_NT_algorithm* self, int p) {
    _Alg* pThis = (_Alg*)self;
    if (p == kParamIrSelectL) loadIr(pThis, pThis->v[kParamIrSelectL], false);
    if (p == kParamIrSelectR) loadIr(pThis, pThis->v[kParamIrSelectR], true);
}

static int parameterString(_NT_algorithm* self, int p, int v, char* buff) {
    _Alg* pThis = (_Alg*)self;
    if ((p == kParamIrSelectL || p == kParamIrSelectR) &&
        pThis->irFolderIndex >= 0 && v >= 0 && v < pThis->irSampleCount) {
        _NT_wavInfo info;
        NT_getSampleFileInfo((uint32_t)pThis->irFolderIndex, (uint32_t)v, info);
        if (info.name) {
            strncpy(buff, info.name, kNT_parameterStringSize-1);
            buff[kNT_parameterStringSize-1] = 0;
            return (int)strlen(buff);
        }
    }
    return 0;
}

// ---------------------------------------------------------------------------
//  Saturation
// ---------------------------------------------------------------------------
static inline float saturate(float s, int type, float drive) {
    if (drive <= 0.f) return s;
    float pregain = 1.f + drive * 7.f;
    float x = s * pregain;
    float sat;
    switch (type) {
        case 1: {
            float thresh = 1.f - drive * 0.6f;
            sat = (x > thresh) ? thresh : (x < -thresh) ? -thresh : x;
        } break;
        case 2:
            sat = tanhf(x * (1.f + fabsf(x) * 0.5f) * 1.5f);
            break;
        case 3: {
            float xc = x * 0.8f;
            if (xc >  1.f) xc =  1.f;
            if (xc < -1.f) xc = -1.f;
            sat = xc - (xc*xc*xc)/3.f + xc*xc*0.1f;
        } break;
        default:
            sat = tanhf(x);
            break;
    }
    return sat / pregain;
}

// ---------------------------------------------------------------------------
//  step
// ---------------------------------------------------------------------------
static void step(_NT_algorithm* self, float* busFrames, int numFramesBy4) {
    _Alg* pThis = (_Alg*)self;
    int numFrames = numFramesBy4 * 4;

    int inBus   = pThis->v[kParamInput]   - 1;
    int outBusL = pThis->v[kParamOutputL] - 1;
    int outBusR = pThis->v[kParamOutputR] - 1;
    if (inBus < 0 || outBusL < 0 || outBusR < 0) return;

    bool mounted = NT_isSdCardMounted();
    if (mounted != pThis->cardMounted) {
        pThis->cardMounted   = mounted;
        pThis->folderScanned = false;
    }
    if (mounted && !pThis->folderScanned)
        scanFolder(pThis, NT_algorithmIndex(self));

    const float* in   = busFrames + inBus   * numFrames;
    float*       outL = busFrames + outBusL * numFrames;
    float*       outR = busFrames + outBusR * numFrames;
    bool replaceL = (pThis->v[kParamOutputLMode] != 0);
    bool replaceR = (pThis->v[kParamOutputRMode] != 0);

    // Parameters computed once per block
    float sr      = (float)NT_globals.sampleRate;
    float inGain  = powf(10.f, pThis->v[kParamInputGain]  / 20.f);
    float outGain = powf(10.f, pThis->v[kParamOutputLevel] / 20.f);
    float drive   = pThis->v[kParamDrive]   / 100.f;
    float width   = pThis->v[kParamWidth]   / 100.f;   // 0=mono, 1=full stereo
    float wet     = pThis->v[kParamDryWet]  / 100.f;   // 0=dry, 1=wet
    float dry     = 1.f - wet;
    int   drvType = pThis->v[kParamDriveType];
    float dcR     = 1.f - 2.f*(float)M_PI*40.f/sr;

    // Noise gate: threshold from 0 (off) to 100 (-60dB to 0dB)
    // Gate=0 means off, Gate=100 means very aggressive
    // Boost: clean linear gain before comp/gate/drive
    float boostGain = powf(10.f, pThis->v[kParamBoost] / 20.f);

    // Compressor: peak detector with instant attack, variable release
    float makeupGain    = powf(10.f, pThis->v[kParamMakeup] / 20.f);
    float compThreshDb  = (float)pThis->v[kParamCompThresh];  // -80..0 dB
    float compRatio     = (float)pThis->v[kParamCompRatio];
    if (compRatio < 1.f) compRatio = 1.f;
    int compRelMs       = pThis->v[kParamCompRelease];
    if (compRelMs < 10) compRelMs = 100;
    // Release coefficient: how fast envelope decays when signal drops
    float compRelCoef   = expf(-1.f / (compRelMs * 0.001f * sr));
    bool  useComp       = (compRatio > 1.f);

    float gateThresh = 0.f;
    float gateAttack = 0.f, gateRelease = 0.f;
    bool  useGate = (pThis->v[kParamGate] > 0 && pThis->v[kParamInput] > 0);
    if (useGate) {
        float gateDb = -60.f + pThis->v[kParamGate] * 54.f / 100.f;
        gateThresh  = powf(10.f, gateDb / 20.f);
        gateAttack  = 1.f - expf(-1.f / (0.001f * sr));  // 1ms attack
        gateRelease = 1.f - expf(-1.f / (0.050f * sr));  // 50ms release
    }

    // Limiter: 0=off, negative value = ceiling in dB
    bool  limActive  = (pThis->v[kParamLimiter] < 0);
    float limCeil    = limActive ? powf(10.f, pThis->v[kParamLimiter] / 20.f) : 1.f;
    float limRelCoef = expf(-1.f / (0.050f * sr));  // 50ms release

    float* irL   = pThis->dram->L.irBuffer;
    float* dlL   = pThis->dram->L.delayLine;
    float* irR   = pThis->dram->R.irBuffer;
    float* dlR   = pThis->dram->R.delayLine;
    int    lenL  = pThis->irLenL,      lenR  = pThis->irLenR;
    float  normL = pThis->irNormGainL, normR = pThis->irNormGainR;
    int    dposL = pThis->delayPosL,   dposR = pThis->delayPosR;
    bool   hasIrL = pThis->irLoadedL && lenL > 0;
    bool   hasIrR = pThis->irLoadedR && lenR > 0;

    float gateEnv = pThis->gateEnv;
    float compEnv = pThis->compEnv;
    float limEnv  = pThis->limEnv;
    float peakIn  = pThis->peakIn  * 0.999f;  // slow decay for display
    float peakOut = pThis->peakOut * 0.999f;

    float tmp[128];
    int done = 0;
    while (done < numFrames) {
        int batch = numFrames - done;
        if (batch > 128) batch = 128;
        for (int i = 0; i < batch; i++) tmp[i] = in[done+i];

        for (int i = 0; i < batch; i++) {
            float raw = tmp[i] * inGain;

            // DC block
            float s = pThis->dcBlock.process(raw, dcR);

            // Boost — clean pre-drive volume
            s *= boostGain;

            // Compressor: instant attack peak detector, variable release
            if (useComp) {
                float level = fabsf(s);
                // Instant attack: envelope jumps up immediately
                if (level > compEnv)
                    compEnv = level;
                else
                    compEnv *= compRelCoef;  // slow release

                // Gain reduction in dB domain
                float envDb = (compEnv > 1e-6f)
                    ? 20.f * log10f(compEnv) : -120.f;
                float gainRedDb = 0.f;
                if (envDb > compThreshDb)
                    gainRedDb = (compThreshDb - envDb) * (1.f - 1.f/compRatio);
                float gainRed = powf(10.f, gainRedDb / 20.f);
                pThis->debugGainRed = gainRed;
                s *= gainRed * makeupGain;
            }

            // Track pre-drive peak for display
            float absS = fabsf(s);
            if (absS > peakIn) peakIn = absS;

            // Noise gate — envelope follower on input
            float gateGain = 1.f;
            if (useGate) {
                if (absS > gateEnv)
                    gateEnv += gateAttack  * (absS - gateEnv);
                else
                    gateEnv += gateRelease * (absS - gateEnv);
                // Hard gate: open above threshold, closed below
                gateGain = (gateEnv > gateThresh) ? 1.f : 0.f;
            }
            s *= gateGain;

            // Saturation
            s = saturate(s, drvType, drive);

            // Left IR
            float wetL = s;
            if (hasIrL) {
                dlL[dposL] = s;
                wetL = 0.f;
                int rpos = dposL;
                for (int k = 0; k < lenL; k++) {
                    wetL += irL[k] * dlL[rpos];
                    if (--rpos < 0) rpos = lenL-1;
                }
                wetL *= normL;
                if (++dposL >= lenL) dposL = 0;
            }

            // Right IR
            float wetR = s;
            if (hasIrR) {
                dlR[dposR] = s;
                wetR = 0.f;
                int rpos = dposR;
                for (int k = 0; k < lenR; k++) {
                    wetR += irR[k] * dlR[rpos];
                    if (--rpos < 0) rpos = lenR-1;
                }
                wetR *= normR;
                if (++dposR >= lenR) dposR = 0;
            }

            // Stereo width: blend L and R IR outputs
            // width=1: full stereo (L=wetL, R=wetR)
            // width=0: mono (L=R=mid)
            float mid  = (wetL + wetR) * 0.5f;
            float outWetL = mid + (wetL - mid) * width;
            float outWetR = mid + (wetR - mid) * width;

            // Dry/wet mix: dry = pre-IR signal (s), wet = IR output
            float finalL = (dry * s + wet * outWetL) * outGain;
            float finalR = (dry * s + wet * outWetR) * outGain;

            // Track output peak for display
            float absOut = fabsf(finalL);
            if (absOut > peakOut) peakOut = absOut;

            // Limiter — instant attack, fast release, applied to both channels
            if (limActive) {
                float peak = fabsf(finalL) > fabsf(finalR) ? fabsf(finalL) : fabsf(finalR);
                if (peak > limEnv) limEnv = peak;
                else limEnv *= limRelCoef;
                if (limEnv > limCeil) {
                    float limGain = limCeil / limEnv;
                    finalL *= limGain;
                    finalR *= limGain;
                }
            }

            outL[done+i] = replaceL ? finalL : outL[done+i] + finalL;
            outR[done+i] = replaceR ? finalR : outR[done+i] + finalR;
        }
        done += batch;
    }

    pThis->delayPosL = dposL;
    pThis->delayPosR = dposR;
    pThis->gateEnv   = gateEnv;
    pThis->compEnv   = compEnv;
    pThis->limEnv    = limEnv;
    pThis->peakIn    = peakIn;
    pThis->peakOut   = peakOut;
}

// ---------------------------------------------------------------------------
//  draw
// ---------------------------------------------------------------------------
static void drawBar(int x, int y, int w, int h, int value, int col) {
    NT_drawShapeI(kNT_box, x, y, x+w, y+h, 5);
    int fill = (value * (w-2)) / 100;
    if (fill > 0)
        NT_drawShapeI(kNT_rectangle, x+1, y+1, x+1+fill, y+h-1, col);
}

// Draw a peak meter bar (value 0.0-1.0 linear)
static void drawMeter(int x, int y, int w, int h, float val) {
    NT_drawShapeI(kNT_box, x, y, x+w, y+h, 5);
    int fill = (int)(val * (w-2));
    if (fill > w-2) fill = w-2;
    if (fill > 0) {
        // Green for low, yellow for mid, red for hot
        int col = (fill < (w-2)*6/10) ? 10 :
                  (fill < (w-2)*9/10) ? 14 : 12;
        NT_drawShapeI(kNT_rectangle, x+1, y+1, x+1+fill, y+h-1, col);
    }
}

static bool draw(_NT_algorithm* self) {
    _Alg* pThis = (_Alg*)self;

    // Title bar
    NT_drawShapeI(kNT_rectangle, 0, 0, 255, 10, 10);
    NT_drawText(128, 8, "Guitar Amp Sim", 15, kNT_textCentre, kNT_textNormal);

    // --- Left section: Amp params ---
    // Input gain
    NT_drawText(2, 19, "IN", 8, kNT_textLeft, kNT_textTiny);
    int inVal = (int)((pThis->v[kParamInputGain] + 20) * 100 / 60);
    if (inVal<0) inVal=0; if (inVal>100) inVal=100;
    drawBar(14, 12, 58, 6, inVal, 12);

    // Boost
    NT_drawText(2, 26, "BS", 8, kNT_textLeft, kNT_textTiny);
    int bsVal = (int)(pThis->v[kParamBoost] * 100 / 30);
    drawBar(14, 19, 58, 6, bsVal, 13);

    // Gate
    NT_drawText(2, 33, "GT", 8, kNT_textLeft, kNT_textTiny);
    drawBar(14, 26, 58, 6, pThis->v[kParamGate], pThis->v[kParamGate]>0 ? 10 : 5);

    // Comp — show threshold as bar, ratio as number
    NT_drawText(2, 40, "CP", 8, kNT_textLeft, kNT_textTiny);
    int cpVal = (int)((pThis->v[kParamCompThresh] + 40) * 100 / 40);
    bool compActive = (pThis->compEnv > powf(10.f, pThis->v[kParamCompThresh]/20.f));
    drawBar(14, 33, 42, 6, cpVal, compActive ? 14 : 8);
    char ratBuf[4]; NT_intToString(ratBuf, pThis->v[kParamCompRatio]);
    NT_drawText(58, 40, ratBuf, 9, kNT_textLeft, kNT_textTiny);

    // Drive
    NT_drawText(2, 47, "DR", 8, kNT_textLeft, kNT_textTiny);
    drawBar(14, 40, 58, 6, pThis->v[kParamDrive], 11);

    // Drive type + Width + Wet
    static const char* dNames[] = { "Soft","Hard","Fuzz","Tape" };
    int dt = pThis->v[kParamDriveType];
    NT_drawText(2, 54, (dt>=0&&dt<=3)?dNames[dt]:"", 9, kNT_textLeft, kNT_textTiny);
    NT_drawText(32, 54, "W", 7, kNT_textLeft, kNT_textTiny);
    drawBar(39, 47, 16, 6, pThis->v[kParamWidth], 9);
    NT_drawText(57, 54, "M", 7, kNT_textLeft, kNT_textTiny);
    drawBar(64, 47, 12, 6, pThis->v[kParamDryWet], 9);

    // --- Divider ---
    NT_drawShapeI(kNT_line, 79, 11, 79, 63, 6);

    // --- Middle section: Level meters ---
    // Debug: show actual v[] values and envelope states
    char dbuf[32];
    NT_drawText(83, 16, "GT:", 7, kNT_textLeft, kNT_textTiny);
    NT_intToString(dbuf, pThis->v[kParamGate]);
    NT_drawText(97, 16, dbuf, 9, kNT_textLeft, kNT_textTiny);

    NT_drawText(83, 24, "CR:", 7, kNT_textLeft, kNT_textTiny);
    NT_intToString(dbuf, pThis->v[kParamCompRatio]);
    NT_drawText(97, 24, dbuf, 9, kNT_textLeft, kNT_textTiny);

    NT_drawText(83, 32, "CT:", 7, kNT_textLeft, kNT_textTiny);
    NT_intToString(dbuf, pThis->v[kParamCompThresh]);
    NT_drawText(97, 32, dbuf, 9, kNT_textLeft, kNT_textTiny);

    NT_drawText(83, 40, "CE:", 7, kNT_textLeft, kNT_textTiny);
    NT_floatToString(dbuf, pThis->compEnv, 3);
    NT_drawText(97, 40, dbuf, 9, kNT_textLeft, kNT_textTiny);

    NT_drawText(83, 48, "GR:", 7, kNT_textLeft, kNT_textTiny);
    NT_floatToString(dbuf, pThis->debugGainRed, 3);
    NT_drawText(97, 48, dbuf, 9, kNT_textLeft, kNT_textTiny);

    NT_drawText(83, 56, "PI:", 7, kNT_textLeft, kNT_textTiny);
    NT_floatToString(dbuf, pThis->peakIn, 3);
    NT_drawText(97, 56, dbuf, 9, kNT_textLeft, kNT_textTiny);

    // --- Divider ---
    NT_drawShapeI(kNT_line, 122, 11, 122, 63, 6);

    // --- Right section: IR names ---
    NT_drawText(125, 19, "CAB", 8, kNT_textLeft, kNT_textTiny);

    if (!pThis->cardMounted) {
        NT_drawText(125, 30, "No SD", 7, kNT_textLeft, kNT_textTiny);
    } else if (pThis->irFolderIndex < 0) {
        NT_drawText(125, 30, "No CabIR", 7, kNT_textLeft, kNT_textTiny);
    } else if (pThis->awaitingLoadL || pThis->awaitingLoadR) {
        NT_drawText(125, 30, "Loading...", 9, kNT_textLeft, kNT_textTiny);
    } else {
        auto showIr = [&](int param, int y, const char* prefix, bool loaded) {
            NT_drawShapeI(kNT_rectangle, 125, y-4, 130, y+1,
                          loaded ? 10 : 5);
            NT_drawText(133, y, prefix, 8, kNT_textLeft, kNT_textTiny);
            int sel = pThis->v[param];
            if (sel >= 0 && sel < pThis->irSampleCount) {
                _NT_wavInfo info;
                NT_getSampleFileInfo((uint32_t)pThis->irFolderIndex,
                                     (uint32_t)sel, info);
                if (info.name)
                    NT_drawText(145, y, info.name, 11,
                                kNT_textLeft, kNT_textTiny);
            }
        };
        showIr(kParamIrSelectL, 27, "L", pThis->irLoadedL);
        showIr(kParamIrSelectR, 37, "R", pThis->irLoadedR);

        // Stereo width indicator
        NT_drawText(125, 50, "W:", 7, kNT_textLeft, kNT_textTiny);
        drawBar(135, 44, 30, 7, pThis->v[kParamWidth], 9);
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
        case kNT_selector_factoryInfo:  return (uintptr_t)((data == 0) ? &factory : NULL);
    }
    return 0;
}
