/****************************************************************************
*
*    Copyright (c) 2005 - 2022 by Vivante Corp.  All rights reserved.
*
*    The material in this file is confidential and contains trade secrets
*    of Vivante Corporation. This is proprietary information owned by
*    Vivante Corporation. No part of this work may be disclosed,
*    reproduced, copied, transmitted, or used in any way for any purpose,
*    without the express written permission of Vivante Corporation.
*
*****************************************************************************/


#ifndef __gc_hal_user_cl_h_
#define __gc_hal_user_cl_h_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 ****************************** Object Declarations ***************************
 ******************************************************************************/
typedef enum {
    OCL_MEM_MODEL_UNIFIED, /* share physical mem       */
    OCL_MEM_MODEL_INDEPENDENT, /* independent physical mem */
    OCL_MEM_MODEL_SVM, /* OCL SVM                  */
} oclMemoryModel;

/* gcoCL_DEVICE_INFO object. */
typedef struct _gcoCL_DEVICE_INFO {
    gctUINT             maxComputeUnits;
    gctUINT             ShaderCoreCount;
    gctUINT             maxWorkItemDimensions;
    gctUINT             maxWorkItemSizes[3];
    gctUINT             maxWorkGroupSize;
    gctUINT64           maxGlobalWorkSize;
    gctUINT             clockFrequency;

    gctUINT             addrBits;
    gctUINT64           maxMemAllocSize;
    gctUINT64           globalMemSize;
    gctUINT64           localMemSize;
    gctUINT             localMemType;               /* cl_device_local_mem_type */
    gctUINT             globalMemCacheType;         /* cl_device_mem_cache_type */
    gctUINT             globalMemCachelineSize;
    gctUINT64           globalMemCacheSize;
    gctUINT             maxConstantArgs;
    gctUINT64           maxConstantBufferSize;
    gctUINT             maxParameterSize;
    gctUINT             memBaseAddrAlign;
    gctUINT             minDataTypeAlignSize;
    gctSIZE_T           maxPrintfBufferSize;

    gctBOOL             imageSupport;
    gctUINT             maxReadImageArgs;
    gctUINT             maxWriteImageArgs;
    gctUINT             vectorWidthChar;
    gctUINT             vectorWidthShort;
    gctUINT             vectorWidthInt;
    gctUINT             vectorWidthLong;
    gctUINT             vectorWidthFloat;
    gctUINT             vectorWidthDouble;
    gctUINT             vectorWidthHalf;
    gctUINT             image2DMaxWidth;
    gctUINT             image2DMaxHeight;
    gctUINT             image3DMaxWidth;
    gctUINT             image3DMaxHeight;
    gctUINT             image3DMaxDepth;
    gctUINT             maxSamplers;

    gctUINT64           queueProperties;          /* cl_command_queue_properties */
    gctBOOL             hostUnifiedMemory;
    gctBOOL             errorCorrectionSupport;
    gctUINT64           halfFpConfig;             /* cl_device_fp_config */
    gctUINT64           singleFpConfig;           /* cl_device_fp_config */
    gctUINT64           doubleFpConfig;           /* cl_device_fp_config */
    gctUINT             profilingTimingRes;
    gctBOOL             endianLittle;
    gctBOOL             deviceAvail;
    gctBOOL             compilerAvail;
    gctBOOL             linkerAvail;
    gctUINT64           execCapability;           /* cl_device_exec_capabilities */
    gctBOOL             atomicSupport;
    gctSIZE_T           imageMaxBufferSize;

    gctBOOL             computeOnlyGpu;           /* without graphih, only compute core GPU flag */
    gctBOOL             supportIMGInstr;          /* support image instruction */
    gctBOOL             psThreadWalker;           /* TW in PS */
    gctBOOL             supportSamplerBaseOffset; /* HW has samplerBaseOffset */
    gctUINT             maxRegisterCount;         /* max temp register count */
    gctBOOL             TxIntegerSupport;
    gctBOOL             halti2;                   /* Halit2 support */
    gctBOOL             multiWGPack;
    gctBOOL             asyncBLT;
    gctBOOL             multiCluster;

    /* cluster info */
    gctBOOL             clusterSupport;
    gctUINT32           clusterCount;  /* cluster count per core */
    gctUINT32           clusterAliveMask[gcdMAX_MAJOR_CORE_COUNT];
    gctUINT32           clusterAliveCount[gcdMAX_MAJOR_CORE_COUNT];
    gctUINT32           clusterMinID[gcdMAX_MAJOR_CORE_COUNT];
    gctUINT32           clusterMaxID[gcdMAX_MAJOR_CORE_COUNT];
    gctUINT32           totalClusterAliveCount;

    gceCHIPMODEL        chipModel;
    gctUINT32           chipRevision;

    gctBOOL             VA64;   /* virtual address is 64 bits */
    gctBOOL             smallBatch;
    gctBOOL             unifiedConst;
    gctUINT32           SRAMSize;
    gctUINT32           extSRAMSize[gcvSRAM_EXT_COUNT];
    gctUINT32           extSRAMCount;
    gctUINT32           vidMemCount;

    gctBOOL             parallelShaderIssue;
    gctUINT             maxRegisterPerThread;

    gceHARDWARE_TYPE    hwType;
    oclMemoryModel      memoryModel;
    gctUINT             deviceIndex; /* device id in hal kernel */
} gcoCL_DEVICE_INFO;

typedef gcoCL_DEVICE_INFO *gcoCL_DEVICE_INFO_PTR;

typedef struct _gcsCommitInfo {
    gceENGINE               engine;
    gctUINT                 coreCount;
    gctADDRESS              cmdPhysical;
    gctUINT32               *cmdLogical;
    gctBOOL                 shared;
    gctBOOL                 mirrored;
    struct {
        gctADDRESS          cmdPhysical;
        gctUINT32*          cmdLogical;
        gctUINT32           cmdSize;
    } subCmd[gcvCORE_3D_MAX];
    gctUINT32               cmdSize;
    gctUINT32               reservedHead;
    gctUINT32               reservedTail;
    gctUINT32               videoMemNode;

    gctBOOL                 contextSwitched;
    gctUINT64               commitStamp;
} gcsCommitInfo;


typedef struct _gcoCL_ALLOC_INFO
{
    struct {
        gctBOOL     bValid;
        gctUINT     extSRAMIndex;
    } extSRAM;

    struct {
        gctBOOL     bValid;
        gctUINT     SRAMIndex;
    } SRAM;

    struct {
        gctBOOL     bValid;
        gctUINT     vidMemIndex;
    } vidMem;
} gcoCL_ALLOC_INFO;


/*******************************************************************************
 **
 **  gcoCL_SetHardwareType
 **
 **  Set hardware type in CL. If the specific type is not available,
 **  it will query the first available type and set it.
 **
 **  INPUT:
 **
 **      The hardware type to be set.
 **
 **  OUTPUT:
 **
 **      Nothing
 */
gceSTATUS
gcoCL_SetHardwareType(IN gceHARDWARE_TYPE Type);

gceSTATUS
gcoCL_ForceSetHardwareType(IN gceHARDWARE_TYPE Type, OUT gceHARDWARE_TYPE *savedType);

gceSTATUS
gcoCL_ForceRestoreHardwareType(IN gceHARDWARE_TYPE savedType);

/*******************************************************************************
 **
 **  gcoCL_GetCurrentPhysicalAddr
 **
 */
gceSTATUS
gcoCL_GetCurrentPhysicalAddr(IN gcsSURF_NODE_PTR Node,
                             IN gceSURF_TYPE     surfType,
                             OUT gctADDRESS     *Address);

/*******************************************************************************
 **
 **  gcoCL_GetPhysicalAddr
 **
 */
gctADDRESS
gcoCL_GetPhysicalAddr(IN gcsSURF_NODE_PTR    Node,
                      IN gcoHARDWARE         Hardware);

/*******************************************************************************
 **
 **  gcoCL_InitializeHardware
 **
 **  Initialize hardware.  This is required for each thread.
 **
 **  INPUT:
 **
 **      Nothing
 **
 **  OUTPUT:
 **
 **      Nothing
 */
gceSTATUS
gcoCL_InitializeHardware();

/**********************************************************************
 **
 **  gcoCL_SetHardware
 **
 **  Set the gcoHARDWARE object for current thread.
 **
 **  INPUT:
 **
 **      Nothing
 **
 **  OUTPUT:
 **
 **      Nothing
 */
gceSTATUS
gcoCL_SetHardware(IN gcoHARDWARE        hw,
                  OUT gcoHARDWARE      *savedHW,
                  OUT gceHARDWARE_TYPE *savedType,
                  OUT gctUINT32        *savedCoreIndex);

gceSTATUS
gcoCL_RestoreContext(IN gcoHARDWARE preHW,
                     IN gceHARDWARE_TYPE preType,
                     IN gctUINT32 preCoreIndex);

#define gcmDECLARE_SWITCHVARS                                                                      \
    gcoHARDWARE      _savedHW        = gcvNULL;                                                    \
    gctBOOL          _switched       = gcvFALSE;                                                   \
    gctUINT32        _savedCoreIndex = 0;                                                          \
    gceHARDWARE_TYPE _savedType      = gcvHARDWARE_INVALID

#define gcmSWITCH_TO_HW(hw)                                                                        \
    {                                                                                              \
        gcoCL_SetHardware((hw), &_savedHW, &_savedType, &_savedCoreIndex);                         \
        _switched = gcvTRUE;                                                                       \
    }

#define gcmSWITCH_TO_DEFAULT()                                                                     \
    {                                                                                              \
        gcoCL_SetHardware(gcvNULL, &_savedHW, &_savedType, &_savedCoreIndex);                      \
        _switched = gcvTRUE;                                                                       \
    }

#define gcmRESTORE_HW()                                                                            \
    {                                                                                              \
        if (_switched) {                                                                           \
            gcoCL_RestoreContext(_savedHW, _savedType, _savedCoreIndex);                           \
            _switched = gcvFALSE;                                                                  \
        }                                                                                          \
    }

#define clmDECLARE_SWITCHVARS()                     \
    gcoHARDWARE _defaultHardware = gcvNULL;         \
    gcoHARDWARE _currentHardware = gcvNULL;         \
    gceHARDWARE_TYPE _currentType;                  \
    gceHARDWARE_TYPE _targetType;                   \
    gctUINT32 _currentDevIndex = 0;                 \
    gcsTLS_PTR _tls;                                \
    gctBOOL    _switch = gcvFALSE

#define clmSWITCH_TO_HW(Hardware)                       \
    gcmVERIFY_OK(gcoOS_GetTLS(&_tls));                  \
    _currentType     = _tls->currentType;               \
    _targetType      = _tls->targetType;                \
    _defaultHardware = _tls->defaultHardware;           \
    _currentHardware = _tls->currentHardware;           \
    _currentDevIndex = _tls->currentDevIndex;           \
                                                        \
    _switch               = gcvTRUE;                    \
    _tls->defaultHardware = Hardware;                   \
    _tls->currentHardware = Hardware;                   \
    _tls->targetType      = Hardware->constructType;    \
    _tls->currentType     = Hardware->constructType;    \
    _tls->currentDevIndex = Hardware->devIndex;


#define clmSWITCH_TO_NULL()                             \
    gcmVERIFY_OK(gcoOS_GetTLS(&_tls));                  \
    _currentType     = _tls->currentType;               \
    _targetType      = _tls->targetType;                \
    _defaultHardware = _tls->defaultHardware;           \
    _currentHardware = _tls->currentHardware;           \
    _currentDevIndex = _tls->currentDevIndex;           \
                                                        \
    _switch               = gcvTRUE;                    \
    _tls->defaultHardware = gcvNULL;                    \
    _tls->currentHardware = gcvNULL;                    \
    _tls->targetType      = gcvHARDWARE_INVALID;        \
    _tls->currentType     = gcvHARDWARE_INVALID;        \
    _tls->currentDevIndex = 0;

#define clmRESTORE_HW()                                 \
    do {                                                \
        if (_switch)                                    \
        {                                               \
            _switch                 = gcvFALSE;         \
            _tls->defaultHardware   = _defaultHardware; \
            _tls->currentHardware   = _currentHardware; \
            _tls->targetType        = _targetType;      \
            _tls->currentType       = _currentType;     \
            _tls->currentDevIndex   = _currentDevIndex; \
        }                                               \
    } while (0);

/*******************************************************************************
 **
 **  gcoCL_AllocateMemory
 **
 **  Allocate memory from the kernel.
 **
 **  INPUT:
 **
 **      gctSIZE_T *Bytes
 **          Pointer to the number of bytes to allocate.
 **
 **  OUTPUT:
 **
 **      gctSIZE_T *Bytes
 **          Pointer to a variable that will receive the aligned number of bytes
 **          allocated.
 **
 **      gctADDRESS *Physical
 **          Pointer to a variable that will receive the gpu virtual address of
 **          the allocated memory, might be same as gpu physical address for flat
 **          mapping case etc.
 **
 **      gctPOINTER *Logical
 **          Pointer to a variable that will receive the logical address of the
 **          allocation.
 **
 **      gcsSURF_NODE_PTR *Node
 **          Pointer to a variable that will receive the gcsSURF_NODE structure
 **          pointer that describes the video memory to lock.
 **
 **     gcePOOL Pool
 **          the pool allocate memory from, gcvPOOL_DEFAULT by default
 */
gceSTATUS
gcoCL_AllocateMemory(IN OUT gctSIZE_T     *Bytes,
                     OUT gctADDRESS       *Physical,
                     OUT gctPOINTER       *Logical,
                     OUT gcsSURF_NODE_PTR *Node,
                     IN gceSURF_TYPE       Type,
                     IN gctUINT32          Flag,
                     IN gcePOOL            Pool);

/*******************************************************************************
 **
 **  gcoCL_AllocateMemoryEx
 **
 **  Allocate contiguous memory from the kernel and lock it for all HW types.
 **
 **  INPUT:
 **
 **      gctSIZE_T *Bytes
 **          Pointer to the number of bytes to allocate.
 **
 **  OUTPUT:
 **
 **      gctSIZE_T *Bytes
 **          Pointer to a variable that will receive the aligned number of bytes
 **          allocated.
 **
 **      gctADDRESS *Physical
 **          Pointer to a variable that will receive the gpu virtual address of
 **          the allocated memory, might be same as gpu physical address for flat
 **          mapping case etc.
 **
 **      gctPOINTER *Logical
 **          Pointer to a variable that will receive the logical address of the
 **          allocation.
 **
 **      gcsSURF_NODE_PTR *Node
 **          Pointer to a variable that will receive the gcsSURF_NODE structure
 **          pointer that describes the video memory to lock.
 **
 **     gcePOOL Pool
 **          the pool allocate memory from, gcvPOOL_DEFAULT by default
 **
 **     gcoHARDWARE *HardwareArray
 **          Pointer to an array that contains all the hardware pointers
 **
 **     gctUINT32 HardwareCnt
 **          count for the hardwares
 **
 **     const gcoCL_EXTRA_INFO *extraInfo
 **          extra info for allocate memory
 */
gceSTATUS
gcoCL_AllocateMemoryEx(
    IN OUT gctSIZE_T *      Bytes,
    OUT gctADDRESS *        Physical,
    OUT gctPOINTER *        Logical,
    OUT gcsSURF_NODE_PTR *  Node,
    IN  gceSURF_TYPE        Type,
    IN  gctUINT32           Flag,
    IN  gcePOOL             Pool,
    IN  gcoHARDWARE *       HardwareArray,
    IN  gctUINT32           HardwareCnt,
    IN  const gcoCL_ALLOC_INFO *extraInfo
    );

gceSTATUS
gcoCL_CreateSubMemory(
    IN gcsSURF_NODE_PTR     node,
    IN gctSIZE_T            offset,
    IN  gctSIZE_T           size,
    OUT gctADDRESS*         physical,
    OUT gctPOINTER*         logical,
    OUT gcsSURF_NODE_PTR*   subNode,
    IN gctUINT32            flags,
    IN gcoHARDWARE*         hardwareArray,
    IN gctUINT32            hardwareCnt);

/*******************************************************************************
 **
 **  gcoCL_FreeMemory
 **
 **  Free contiguous memory to the kernel.
 **
 **  INPUT:
 **
 **      gctUINT32 Physical
 **          The gpu virutal addresses of the allocated pages.
 **
 **      gctPOINTER Logical
 **          The logical address of the allocation.
 **
 **      gctSIZE_T Bytes
 **          Number of bytes allocated.
 **
 **      gcsSURF_NODE_PTR Node
 **          Pointer to a gcsSURF_NODE structure
 **          that describes the video memory to unlock.
 **
 **  OUTPUT:
 **
 **      Nothing
 */
gceSTATUS
gcoCL_FreeMemory(IN gctADDRESS       Physical,
                 IN gctPOINTER       Logical,
                 IN gctSIZE_T        Bytes,
                 IN gcsSURF_NODE_PTR Node,
                 IN gceSURF_TYPE     Type);

gceSTATUS
gcoCL_SyncFreeMemory(IN gctADDRESS       Physical,
                     IN gctPOINTER       Logical,
                     IN gctUINT          Bytes,
                     IN gcsSURF_NODE_PTR Node,
                     IN gceSURF_TYPE     Type);

/*******************************************************************************
 **
 **  gcoCL_WrapUserMemory
 **
 */
gceSTATUS
gcoCL_WrapUserMemory(IN gctPOINTER         Ptr,
                     IN gctSIZE_T          Bytes,
                     IN gctBOOL            VIVUnCached,
                     OUT gctADDRESS       *Physical,
                     OUT gcsSURF_NODE_PTR *Node,
                     IN gctUINT32          Flag);

/*******************************************************************************
 **
 **  gcoCL_WrapUserMemoryEx
 **
 */
gceSTATUS
gcoCL_WrapUserMemoryEx(IN gctPOINTER           Ptr,
                       IN gctSIZE_T            Bytes,
                       IN gctBOOL              VIVUnCached,
                       OUT gctADDRESS         *Physical,
                       OUT gcsSURF_NODE_PTR   *Node,
                       IN gctUINT32            Flag,
                       IN gcoHARDWARE         *HardwareArray,
                       IN gctUINT32            HardwareCnt);

/*******************************************************************************
 **
 **  gcoCL_ImportDmabuffer
 **  1.wrap dma buffer
 **  2.lock devices
 */
gceSTATUS
gcoCL_ImportDmabuffer(IN  gctUINT32           Fd,
                      IN  gctSIZE_T           Bytes,
                      OUT gctADDRESS         *Physical,
                      OUT gcsSURF_NODE_PTR   *Node,
                      OUT gctPOINTER         *Logical,
                      IN  gctUINT32           Flag, /*gcvALLOC_FLAG_DMABUF*/
                      IN  gcoHARDWARE        *HardwareArray,
                      IN  gctUINT32           HardwareCnt);

/*******************************************************************************
 **
 **  gcoCL_WrapUserPhysicalMemory
 **
 */
gceSTATUS
gcoCL_WrapUserPhysicalMemory(IN gctUINT32_PTR      Physical,
                             IN gctSIZE_T          Bytes,
                             IN gctBOOL            VIVUnCached,
                             OUT gctPOINTER       *Logical,
                             OUT gctADDRESS       *Address,
                             OUT gcsSURF_NODE_PTR *Node,
                             IN gctUINT32          Flag);

gceSTATUS
gcoCL_WrapUserPhysicalMemoryEx(IN gctUINT32_PTR        Physical,
                               IN gctSIZE_T            Bytes,
                               IN gctBOOL              VIVUnCached,
                               OUT gctPOINTER         *Logical,
                               OUT gctADDRESS         *Address,
                               OUT gcsSURF_NODE_PTR   *Node,
                               IN gctUINT32            Flag,
                               IN  gcoHARDWARE        *HardwareArray,
                               IN  gctUINT32           HardwareCnt);
/*******************************************************************************
 **
 **  gcoCL_FlushMemory
 **
 **  Flush memory to the kernel.
 **
 **  INPUT:
 **
 **      gcsSURF_NODE_PTR  Node
 **          Pointer to a gcsSURF_NODE structure
 **          that describes the video memory to flush.
 **
 **      gctPOINTER Logical
 **          The logical address of the allocation.
 **
 **      gctSIZE_T Bytes
 **          Number of bytes allocated.
 **
 **  OUTPUT:
 **
 **      Nothing
 */
gceSTATUS
gcoCL_FlushMemory(IN gcsSURF_NODE_PTR Node, IN gctPOINTER Logical, IN gctSIZE_T Bytes);

/*******************************************************************************
 **
 **  gcoCL_InvalidateMemoryCache
 **
 **  Invalidate memory cache in CPU.
 **
 **  INPUT:
 **
 **      gcsSURF_NODE_PTR  Node
 **          Pointer to a gcsSURF_NODE structure
 **          that describes the video memory to flush.
 **
 **      gctPOINTER Logical
 **          The logical address of the allocation.
 **
 **      gctSIZE_T Bytes
 **          Number of bytes allocated.
 **
 **  OUTPUT:
 **
 **      Nothing
 */
gceSTATUS
gcoCL_InvalidateMemoryCache(IN gcsSURF_NODE_PTR Node,
                            IN gctPOINTER Logical, IN gctSIZE_T Bytes);

gceSTATUS
gcoCL_MemoryCache(
    IN gcoHARDWARE*         hardware,
    IN gctUINT              hardwareCount,
    IN gcsSURF_NODE_PTR     node,
    IN gctPOINTER           logical,
    IN gctSIZE_T            bytes,
    IN gctUINT32            flags,
    IN gceCACHEOPERATION    operation
);
/*******************************************************************************
 **
 **  gcoCL_ShareMemoryWithStream
 **
 **  Share memory with a stream.
 **
 **  INPUT:
 **
 **      gcoSTREAM Stream
 **          Pointer to the stream object.
 **
 **  OUTPUT:
 **
 **      gctSIZE_T *Bytes
 **          Pointer to a variable that will receive the aligned number of bytes
 **          allocated.
 **
 **      gctADDRESS *Physical
 **          Pointer to a variable that will receive the gpu virtual addresses of
 **          the allocated memory.
 **
 **      gctPOINTER *Logical
 **          Pointer to a variable that will receive the logical address of the
 **          allocation.
 **
 **      gcsSURF_NODE_PTR *Node
 **          Pointer to a variable that will receive the gcsSURF_NODE structure
 **          pointer that describes the video memory to lock.
 */
gceSTATUS
gcoCL_ShareMemoryWithStream(IN gcoSTREAM         Stream,
                            OUT gctSIZE_T        *Bytes,
                            OUT gctADDRESS       *Physical,
                            OUT gctPOINTER       *Logical,
                            OUT gcsSURF_NODE_PTR *Node);

/*******************************************************************************
 **
 **  gcoCL_ShareMemoryWithBufObj
 **
 **  Share memory with a BufObj.
 **
 **  INPUT:
 **
 **      gcoBUFOBJ Stream
 **          Pointer to the stream object.
 **
 **  OUTPUT:
 **
 **      gctSIZE_T *Bytes
 **          Pointer to a variable that will receive the aligned number of bytes
 **          allocated.
 **
 **      gctADDRESS *Physical
 **          Pointer to a variable that will receive the gpu virtual addresses of
 **          the allocated memory.
 **
 **      gctPOINTER *Logical
 **          Pointer to a variable that will receive the logical address of the
 **          allocation.
 **
 **      gcsSURF_NODE_PTR *Node
 **          Pointer to a variable that will receive the gcsSURF_NODE structure
 **          pointer that describes the video memory to lock.
 */
gceSTATUS
gcoCL_ShareMemoryWithBufObj(IN gcoBUFOBJ         BufObj,
                            OUT gctSIZE_T        *Bytes,
                            OUT gctADDRESS       *Physical,
                            OUT gctPOINTER       *Logical,
                            OUT gcsSURF_NODE_PTR *Node);

/*******************************************************************************
 **
 **  gcoCL_UnshareMemory
 **
 **  Unshare memory with a stream.
 **
 **  INPUT:
 **
 **      gcsSURF_NODE_PTR  Node
 **          Pointer to a  gcsSURF_NODE structure
 */
gceSTATUS
gcoCL_UnshareMemory(IN gcsSURF_NODE_PTR Node);


gceSTATUS
gcoCL_ShareMemoryWithBufObjEx(
    IN gcoHARDWARE          Hardware,
    IN gcoBUFOBJ            BufObj,
    OUT gctSIZE_T*          Bytes,
    OUT gctADDRESS*         Physical,
    OUT gctPOINTER*         Logical,
    OUT gcsSURF_NODE_PTR*   Node);

gceSTATUS
gcoCL_UnshareMemoryEx(
    IN gcoHARDWARE      Hardware,
    IN gcsSURF_NODE_PTR Node);

/*******************************************************************************
 **
 **  gcoCL_FlushSurface
 **
 **  Flush surface to the kernel.
 **
 **  INPUT:
 **
 **      gcoSURF           Surface
 **          gcoSURF structure
 **          that describes the surface to flush.
 **
 **  OUTPUT:
 **
 **      Nothing
 */
gceSTATUS
gcoCL_FlushSurface(IN gcoSURF Surface);

gceSTATUS
gcoCL_LockSurface(IN gcoSURF Surface, OUT gctADDRESS *Address, OUT gctPOINTER *Memory);

gceSTATUS
gcoCL_UnlockSurface(IN gcoSURF Surface, IN gctPOINTER Memory);

/*******************************************************************************
 **
 **  gcoCL_CreateTexture
 **
 **  Create texture for image.
 **
 **  INPUT:
 **
 **      gctUINT Width
 **          Width of the image.
 **
 **      gctUINT Heighth
 **          Heighth of the image.
 **
 **      gctUINT Depth
 **          Depth of the image.
 **
 **      gctCONST_POINTER Memory
 **          Pointer to the data of the input image.
 **
 **      gctUINT Stride
 **          Size of one row.
 **
 **      gctUINT Slice
 **          Size of one plane.
 **
 **      gceSURF_FORMAT FORMAT
 **          Format of the image.
 **
 **      gceENDIAN_HINT EndianHint
 **          Endian needed to handle the image data.
 **
 **  OUTPUT:
 **
 **      gcoTEXTURE *Texture
 **          Pointer to a variable that will receive the gcoTEXTURE structure.
 **
 **      gcoSURF *Surface
 **          Pointer to a variable that will receive the gcoSURF structure.
 **
 **      gctADDRESS *Physical
 **          Pointer to a variable that will receive the gpu virtual addresses of
 **          the allocated memory.
 **
 **      gctPOINTER *Logical
 **          Pointer to a variable that will receive the logical address of the
 **          allocation.
 **
 **      gctUINT *SurfStride
 **          Pointer to a variable that will receive the stride of the texture.
 */
gceSTATUS
gcoCL_CreateTexture(IN OUT gceIMAGE_MEM_TYPE *MapHostMemory,
                    IN gctUINT                Width,
                    IN gctUINT                Height,
                    IN gctUINT                Depth,
                    IN gctCONST_POINTER       Memory,
                    IN gctUINT                Stride,
                    IN gctUINT                Slice,
                    IN gceSURF_FORMAT         Format,
                    IN gceENDIAN_HINT         EndianHint,
                    OUT gcoTEXTURE            *Texture,
                    OUT gcoSURF               *Surface,
                    OUT gctADDRESS            *Physical,
                    OUT gctPOINTER            *Logical,
                    OUT gctUINT               *SurfStride,
                    OUT gctUINT               *SurfSliceSize);

/*******************************************************************************
 **
 **  gcoCL_DestroyTexture
 **
 **  Destroy an gcoTEXTURE object.
 **
 **  INPUT:
 **
 **      gcoTEXTURE Texture
 **          Pointer to an gcoTEXTURE object.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
gceSTATUS
gcoCL_DestroyTexture(IN gcoTEXTURE Texture, IN gcoSURF Surface);

gceSTATUS
gcoCL_SetupTexture(IN gcoTEXTURE         Texture,
                   IN gcoSURF            Surface,
                   IN gctUINT            SamplerNum,
                   gceTEXTURE_ADDRESSING AddressMode,
                   gceTEXTURE_FILTER     FilterMode);

/*******************************************************************************
 **
 **  gcoCL_QueryDeviceInfo
 **
 **  Query the OpenCL capabilities of the device.
 **
 **  INPUT:
 **
 **      gcoHARDWARE  Hardware
 **          Pointer to the hardware
 **
 **  OUTPUT:
 **
 **      gcoCL_DEVICE_INFO_PTR DeviceInfo
 **          Pointer to the device information
 */
gceSTATUS
gcoCL_QueryDeviceInfo(IN gcoHARDWARE Hardware, OUT gcoCL_DEVICE_INFO_PTR DeviceInfo);

gceSTATUS
gcoCL_QueryDeviceCount(IN gceHARDWARE_TYPE hwType,
                       OUT gctUINT32       *DeviceCount,
                       OUT gctUINT32       *GPUCountPerDevice);

gceSTATUS
gcoCL_QueryDeviceCountWithGPUType(OUT gctUINT32 *DeviceCount,
                                  OUT gctUINT32 *GPUCountPerDevice);

gceSTATUS
gcoCL_QueryDeviceCountWithVIPType(OUT gctUINT32 *DeviceCount,
                                  OUT gctUINT32 *GPUCountPerDevice);

gceSTATUS
gcoCL_QueryDeviceCountWith3D2DType(OUT gctUINT32 *DeviceCount,
                                   OUT gctUINT32 *GPUCountPerDevice);

gceSTATUS
gcoCL_CreateHW(IN gctUINT32 DeviceId, OUT gcoHARDWARE *Hardware);

gceSTATUS
gcoCL_CreateHWWithType(IN gceHARDWARE_TYPE hwType,
                       IN gctUINT32        DeviceId,
                       OUT gcoHARDWARE     *Hardware);

gceSTATUS
gcoCL_DestroyHW(gcoHARDWARE Hardware, gctBOOL Stall);

gceSTATUS
gcoCL_DestroyHWEx(gcoHARDWARE Hardware, gctBOOL Stall);

gceSTATUS
gcoCL_GetHWConfigGpuCount(gctUINT32 *GpuCount);
/*******************************************************************************
 **
 **  gcoCL_Commit
 **
 **  Commit the current command buffer to hardware and optionally wait until the
 **  hardware is finished.
 **
 **  INPUT:
 **
 **      gctBOOL Stall
 **          gcvTRUE if the thread needs to wait until the hardware has finished
 **          executing the committed command buffer.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
gceSTATUS
gcoCL_Commit(IN gctBOOL Stall);

gceSTATUS
gcoCL_CommitWithHardware(IN gcoHARDWARE Hardware, IN gctBOOL Stall);

gceSTATUS
gcoCL_Flush(IN gctBOOL Stall);

gceSTATUS
gcoCL_FlushWithHardware(IN gcoHARDWARE Hardware, IN gctBOOL Stall);

/*******************************************************************************
 **
 **  gcoCL_CreateSignal
 **
 **  Create a new signal.
 **
 **  INPUT:
 **
 **      gctBOOL ManualReset
 **          If set to gcvTRUE, gcoOS_Signal with gcvFALSE must be called in
 **          order to set the signal to nonsignaled state.
 **          If set to gcvFALSE, the signal will automatically be set to
 **          nonsignaled state by gcoOS_WaitSignal function.
 **
 **  OUTPUT:
 **
 **      gctSIGNAL *Signal
 **          Pointer to a variable receiving the created gctSIGNAL.
 */
gceSTATUS
gcoCL_CreateSignal(IN gctBOOL ManualReset, OUT gctSIGNAL *Signal);

/*******************************************************************************
 **
 **  gcoCL_DestroySignal
 **
 **  Destroy a signal.
 **
 **  INPUT:
 **
 **      gctSIGNAL Signal
 **          Pointer to the gctSIGNAL.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
gceSTATUS
gcoCL_DestroySignal(IN gctSIGNAL Signal);

gceSTATUS
gcoCL_SubmitSignal(IN gctSIGNAL Signal, IN gctHANDLE Processs, IN gceENGINE Engine);

/*******************************************************************************
 **
 **  gcoCL_WaitSignal
 **
 **  Wait for a signal to become signaled.
 **
 **  INPUT:
 **
 **      gctSIGNAL Signal
 **          Pointer to the gctSIGNAL.
 **
 **      gctUINT32 Wait
 **          Number of milliseconds to wait.
 **          Pass the value of gcvINFINITE for an infinite wait.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
gceSTATUS
gcoCL_WaitSignal(IN gctSIGNAL Signal, IN gctUINT32 Wait);

/*******************************************************************************
 **
 **  gcoCL_SetSignal
 **
 **  Make a signal to become signaled.
 **
 **  INPUT:
 **
 **      gctSIGNAL Signal
 **          Pointer to the gctSIGNAL.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
gceSTATUS
gcoCL_SetSignal(IN gctSIGNAL Signal);
/*******************************************************************************
 **                                gcoCL_LoadKernel
 ********************************************************************************
 **
 **  Load a pre-compiled and pre-linked kernel program into the hardware.
 **
 **  INPUT:
 **
 **      gcsPROGRAM_STATE *ProgramState
 **          Program state pointer.
 */
gceSTATUS
gcoCL_LoadKernel(IN gcsPROGRAM_STATE *ProgramState);

gceSTATUS
gcoCL_InvokeKernel(IN gctUINT   WorkDim,
                   IN size_t    GlobalWorkOffset[3],
                   IN size_t    GlobalScale[3],
                   IN size_t    GlobalWorkSize[3],
                   IN size_t    LocalWorkSize[3],
                   IN gctUINT   ValueOrder,
                   IN gctBOOL   BarrierUsed,
                   IN gctUINT32 MemoryAccessFlag,
                   IN gctBOOL   bDual16);

gceSTATUS
gcoCL_InvokeThreadWalker(IN gcsTHREAD_WALKER_INFO_PTR Info);

gceSTATUS
gcoCL_MemBltCopy(IN gctUINT32 SrcAddress,
                 IN gctUINT32 DestAddress,
                 IN gctUINT32 CopySize,
                 IN gceENGINE engine);

gceSTATUS
gcoCL_MemWaitAndGetFence(IN gcsSURF_NODE_PTR Node,
                         IN gceENGINE        Engine,
                         IN gceFENCE_TYPE    GetType,
                         IN gceFENCE_TYPE    WaitType);

gceSTATUS
gcoCL_MemWaitAndGetFenceWithHardware(IN gcsSURF_NODE_PTR Node,
                                     IN gcoHARDWARE      Hardware,
                                     IN gceENGINE        Engine,
                                     IN gceFENCE_TYPE    GetType,
                                     IN gceFENCE_TYPE    WaitType);

gceSTATUS
gcoCL_ChooseBltEngine(IN gcsSURF_NODE_PTR node, OUT gceENGINE *engine);

gceSTATUS
gcoCL_IsFeatureAvailable(IN gcoHARDWARE Hardware, IN gceFEATURE Feature);

gctUINT
gcoCL_QueryBLTFenceEndianHint(IN gcoHARDWARE Hardware);

gceSTATUS
gcoCL_3dBltLock(IN gcoHARDWARE   Hardware,
                IN gceENGINE     Engine,
                IN gctBOOL       forceSingle,
                IN gctUINT32_PTR *Memory);

gceSTATUS
gcoCL_3dBltUnlock(IN gcoHARDWARE   Hardware,
                  IN gceENGINE     Engine,
                  IN gctBOOL       forceSingle,
                  IN gctUINT32_PTR *Memory);

gceSTATUS
gcoCL_SubmitCmdBuffer(gcoHARDWARE Hardware, uint32_t *states, uint32_t count);

gceSTATUS
gcoCL_FlushCmdBuffer(gcoHARDWARE Hardware, gceENGINE engine, uint32_t *states, uint32_t count);

gctUINT
gcoCL_coreIdToChip(gcoHARDWARE Hardware, gctUINT coreId);

gceSTATUS
gcoCL_MultiGPUSync(IN gcoHARDWARE Hardware, IN gctUINT32_PTR *Memory);

gceSTATUS
gcoCL_MultiGPUFlushCache(IN gcoHARDWARE Hardware, IN gctUINT32_PTR *Memory);

gceSTATUS
gcoCL_FlushCache(IN gcoHARDWARE Hardware, IN gctUINT32_PTR *Memory);

gceSTATUS
gcoCL_SetDefaultHardware(IN gcoHARDWARE hw, OUT gcoHARDWARE *savedHW);

gceSTATUS
gcoCL_SetTimeOut(IN gcoHARDWARE Hardware, IN gctUINT32 timeOut);

gceSTATUS
gcoCL_GetProductName(IN gcoHARDWARE hardware,
                     OUT gctSTRING *ProductName, OUT gctUINT *PID);

gceSTATUS
gcoCL_GetPatchID(IN gcoHARDWARE hardware, OUT gcePATCH_ID *PatchID);

gceSTATUS
gcoCL_CommitCmdBuffer(IN  gcoHARDWARE    Hardware,
                      IN  gceENGINE      engine,
                      OUT gcsCommitInfo *info);

gceSTATUS
gcoCL_MultiGPUAffinityConfig();

gceSTATUS
gcoCL_GetOCLDeviceCount(gctUINT32 *devCount);

gceSTATUS
gcoCL_CreateHWEx(IN gctUINT32     DeviceId,
                 OUT gcoHARDWARE *Hardware);

gceSTATUS
gcoCL_EnableLocalMemory(
    IN gcoHARDWARE Hardware,
    INOUT PVSC_HW_CONFIG pVscHwCfg);

gceSTATUS
gcoCL_GetUscCacheRatio(
    IN gcoHARDWARE Hardware,
    OUT gctUINT32 *uscOclL1CacheRatio,
    OUT gctUINT32 *uscOclAttribCacheRatio);

typedef struct _HardwareSwitchContext *gcoCL_HwSwitchContext;

gceSTATUS
gcoCL_SwitchHardware(
    OUT gcoCL_HwSwitchContext* context,
    IN  gcoHARDWARE hardware);

gceSTATUS
gcoCL_RestoreHardware(
    IN gcoCL_HwSwitchContext context);

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_user_cl_h_ */



