#ifndef __VX_KHR_COMPATIBLE_H__
#define __VX_KHR_COMPATIBLE_H__
/*
 VX_DECONVOLUTION_WEIGHT_LAYOUT_COMPATIBLE_KHRONOS is used to distingush deconvolution weight layout
 [value]
 0: weight_layout is whnc
 1: weight_layout is whcn
*/
#ifndef VX_DECONVOLUTION_WEIGHT_LAYOUT_COMPATIBLE_KHRONOS
#define VX_DECONVOLUTION_WEIGHT_LAYOUT_COMPATIBLE_KHRONOS 1
#endif
/*
 VX_CONVERT_POLICY_WRAP_ENABLE is used to differentiate two overflow_policys(VX_CONVERT_POLICY_WRAP and VX_CONVERT_POLICY_SAT)
 [value]
 0: both overflow_policys considered as VX_CONVERT_POLICY_SAT
 1: overflow_policy is determined by arguments.
*/
#ifndef VX_CONVERT_POLICY_WRAP_ENABLE
#define VX_CONVERT_POLICY_WRAP_ENABLE 1
#endif

#ifndef VX_13_NN_COMPATIBLITY
#define VX_13_NN_COMPATIBLITY 1
#endif
/*
 VX_L2NORM_AXIS_PARAMETER_SUPPORT is used to declare that L2NORMALIZE can support axis parameter
 [value]
 0: not support
 1: support
*/
#ifndef VX_L2NORM_AXIS_PARAMETER_SUPPORT
#define VX_L2NORM_AXIS_PARAMETER_SUPPORT 1
#endif
/*
 VX_SOFTMAX_AXIS_PARAMETER_SUPPORT is used to declare that SOFTAMX can support axis parameter
 [value]
 0: not support
 1: support
*/
#ifndef VX_SOFTMAX_AXIS_PARAMETER_SUPPORT
#define VX_SOFTMAX_AXIS_PARAMETER_SUPPORT 1
#endif
/*
 VX_NORMALIZATION_AXIS_PARAMETER_SUPPORT is used to declare that NORMALIZATION can support axis parameter
 [value]
 0: not support
 1: support
*/
#ifndef VX_NORMALIZATION_AXIS_PARAMETER_SUPPORT
#define VX_NORMALIZATION_AXIS_PARAMETER_SUPPORT 1
#endif
/*
 VX_ACTIVATION_EXT_SUPPORT is used to declare that ACTIVATION can support swish and hswish
 [value]
 0: not support
 1: support
*/
#ifndef VX_ACTIVATION_EXT_SUPPORT
#define VX_ACTIVATION_EXT_SUPPORT 1
#endif

/*
 VX_HARDWARE_CAPS_PARAMS_EXT_SUPPORT is used to query more hardware parameter such as shader sub-group size.
 [value]
 0: not support
 1: support
*/
#ifndef VX_HARDWARE_CAPS_PARAMS_EXT_SUPPORT
#define VX_HARDWARE_CAPS_PARAMS_EXT_SUPPORT 1
#endif

/*
 VX_VA40_EXT_SUPPORT is used to declare that openvx can support VA40.
 [value]
 0: not support
 1: support
*/
#ifndef VX_VA40_EXT_SUPPORT
#define VX_VA40_EXT_SUPPORT 0
#endif

/*
 VX_USER_LOOKUP_TABLE_SUPPORT is used to declare that openvx can support user lookuptable.
 [value]
 0: not support
 1: support
*/
#ifndef VX_USER_LOOKUP_TABLE_SUPPORT
#define VX_USER_LOOKUP_TABLE_SUPPORT 1
#endif

/*
VX_PRELOAD_CONST_TENSOR_SUPPORT is used to declare that openvx can support preload weight/bias and const tensor
 [value]
 0: not support
 1: support(NN conv and TP FC weightbias, and SH const tensor)
*/
#ifndef VX_PRELOAD_CONST_TENSOR_SUPPORT
#define VX_PRELOAD_CONST_TENSOR_SUPPORT 1
#endif

/*
VX_CREATE_TENSOR_SUPPORT_PHYSICAL is used to declare that openvx can support physical address for vxCreateTensorFromHandle
 [value]
 0: not support
 1: support
*/
#ifndef VX_CREATE_TENSOR_SUPPORT_PHYSICAL
#define VX_CREATE_TENSOR_SUPPORT_PHYSICAL 1
#endif

/*
 VX_GRAPH_PREEMPTION_SUPPORT is used to declare that openvx can support different graph preemption function.
 [value]
 0: not support
 1: support
*/
#ifndef VX_GRAPH_PREEMPTION_SUPPORT
#define VX_GRAPH_PREEMPTION_SUPPORT 1
#endif

/*
VX_BATCH_GEMM_API_SUPPORT is used to declare that vsi openvx driver can support vxBatchGemmNode API to transform gemm to convolution
 [value]
 0: not support
 1: support
*/
#ifndef VX_BATCH_GEMM_API_SUPPORT
#define VX_BATCH_GEMM_API_SUPPORT 1
#endif

/*
VX_CONV_3D_API_SUPPORT is used to declare that vsi openvx driver can support conv3d by vxConv3dLayer API.
 [value]
 0: not support
 1: support
*/
#ifndef VX_CONV_3D_API_SUPPORT
#define VX_CONV_3D_API_SUPPORT 1
#endif

/*
VX_DECONV_3D_API_SUPPORT is used to declare that vsi openvx driver can support deconv3d by vxDeconv3dLayer API.
 [value]
 0: not support
 1: support
*/
#ifndef VX_DECONV_3D_API_SUPPORT
#define VX_DECONV_3D_API_SUPPORT 0
#endif

/*
 VX_PAD_CONST_SUPPORT is used to declare that openvx can support pad_const for tensorpad and convolution.
 [value]
 0: not support
 1: support
*/
#ifndef VX_PAD_CONST_SUPPORT
#define VX_PAD_CONST_SUPPORT 1
#endif

/*
 VX_TENSOR_STRIDE_X_BITS_SUPPORT is used to declare that openvx can support tensor which bits of stride in x dimension is not an integer number of bytes.
 [value]
 0: not support
 1: support
*/
#ifndef VX_TENSOR_STRIDE_X_BITS_SUPPORT
#define VX_TENSOR_STRIDE_X_BITS_SUPPORT 1
#endif

/*
VX_REMOVE_RESHAPE_SUPPORT is used to declare if graph opt support to remove reshape op, if support, it's not need to remove reshape in ovxlib.
 0: not support
 1: support
*/
/*
#ifndef VX_REMOVE_RESHAPE_SUPPORT
#define VX_REMOVE_RESHAPE_SUPPORT 0
#endif
*/

/*
VX_STREAM_PROCESSOR_SUPPORT is used to declare that vsi openvx driver can support vxStreamProcessorNode API
 [value]
 0: not support
 1: support
*/
#ifndef VX_STREAM_PROCESSOR_SUPPORT
#define VX_STREAM_PROCESSOR_SUPPORT 0
#endif

/*
 VX_TENSOR_MEMORY_CONNECT_DMA_CHANNEL is used to declare that this tensor connect to fixed DMA channel.
 [value]
 0: not support
 1: support
*/
#ifndef VX_TENSOR_MEMORY_CONNECT_DMA_CHANNEL
#define VX_TENSOR_MEMORY_CONNECT_DMA_CHANNEL 1
#endif

/*
 VX_SCALE_EXTRA_PARAMETER_SUPPORT is used to declare that RESIZE can support align_cornor and half_pixel_center parameter
 [value]
 0: not support
 1: support
*/
#ifndef VX_SCALE_EXTRA_PARAMETER_SUPPORT
#define VX_SCALE_EXTRA_PARAMETER_SUPPORT 1
#endif

#endif /* __VX_KHR_COMPATIBLE_H__ */
