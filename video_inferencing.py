import tensorrt as trt
import jetson.utils
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda 
import time
import PIL

import cv2
import torchvision.transforms as transforms
import torch

import rospy
from std_msgs.msg import String

normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])
                                 
preprocessing = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Scale(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    normalize,
])
	
#gstreamer

model_path = "resnet18.onnx"
height = 360
width = 640
input_size = 224
camera = '/dev/video0'

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

# create camera device
camera = jetson.utils.gstCamera(width, height, camera)

# create display windowdisplay = jetson.utils.glDisplay()
display = jetson.utils.glDisplay()

def simple_cpu_image_hash(image, size=224, channels=4, alpha_black=True) -> bytes:

    background = np.zeros if alpha_black else np.ones

    # get a numpy array with the array in shared gpu/cpu memory
    # there is no copy involved in creation of the numpy array
    in_arr = jetson.utils.cudaToNumpy(*image, channels)  # type: np.ndarray

    # flatten rgba image on black background
    out_arr = skimage.color.rgba2rgb(in_arr,  # in image over
                                     background(shape=3, dtype=np.uint8)),

    # convert to greyscale
    #out_arr = skimage.color.rgb2gray(out_arr)

    # resize to size * size using linear interpolation, aa off since pointless
    out_arr = skimage.transform.resize(out_arr[0], (size, size), anti_aliasing=False)
    return out_arr
# open the camera for streaming
camera.Open()

def build_engine(model_path):
    with trt.Builder(TRT_LOGGER) as builder, \
        builder.create_network() as network, \
        trt.OnnxParser(network, TRT_LOGGER) as parser: 
        builder.max_workspace_size = 1<<20
        builder.max_batch_size = 1
        with open(model_path, "rb") as f:
            parser.parse(f.read())
        engine = builder.build_cuda_engine(network)
        return engine

def alloc_buf(engine):
    # host cpu mem
    h_in_size = trt.volume(engine.get_binding_shape(0))
    h_out_size = trt.volume(engine.get_binding_shape(1))
    h_in_dtype = trt.nptype(engine.get_binding_dtype(0))
    h_out_dtype = trt.nptype(engine.get_binding_dtype(1))
    in_cpu = cuda.pagelocked_empty(h_in_size, h_in_dtype)
    out_cpu = cuda.pagelocked_empty(h_out_size, h_out_dtype)
    # allocate gpu mem
    in_gpu = cuda.mem_alloc(in_cpu.nbytes)
    out_gpu = cuda.mem_alloc(out_cpu.nbytes)
    stream = cuda.Stream()
    return in_cpu, out_cpu, in_gpu, out_gpu, stream


def inference(engine, context, inputs, out_cpu, in_gpu, out_gpu, stream):
    # async version
    # with engine.create_execution_context() as context:  # cost time to initialize
    # cuda.memcpy_htod_async(in_gpu, inputs, stream)
    # context.execute_async(1, [int(in_gpu), int(out_gpu)], stream.handle, None)
    # cuda.memcpy_dtoh_async(out_cpu, out_gpu, stream)
    # stream.synchronize()

    # sync version
    cuda.memcpy_htod(in_gpu, inputs)
    context.execute(1, [int(in_gpu), int(out_gpu)])
    cuda.memcpy_dtoh(out_cpu, out_gpu)
    return out_cpu


        
# open the camera for streaming
camera.Open()
engine = build_engine(model_path)
context = engine.create_execution_context()
# capture frames until user exits
while display.IsOpen():
    t1 = time.time()
    k = camera.CaptureRGBA(zeroCopy=1)
    image, width, height = k
    #img_arr = simple_cpu_image_hash(k)
    #outputs = rep.run(np.random.randn(10, 3, 224, 224).astype(np.float32))
    #img_arr =  simple_cpu_image_hash(k, size = 224)
    img_arr =  jetson.utils.cudaToNumpy(image, width, height, 4)
    rgbImage = cv2.cvtColor(img_arr, cv2.COLOR_RGBA2BGR)
     
    img = cv2.resize(rgbImage, (256,256), interpolation = cv2.INTER_NEAREST)
    #img = cv2.fastNlMeansDenoisingColored(img,None,10,10,7,21)    
    img = img/255    
   	        
    #Channel last to channel first
    img = np.moveaxis(img, 2,0)
    img = np.ascontiguousarray(img, dtype=np.float32)
    img_torch = torch.from_numpy(img)
    #PyTorch preprocessing 
    test_img = preprocessing(img_torch).float()
    final_img = test_img.numpy()
    
    #img_arr = img_arr[:,:,:3].T
    #inputs = np.random.random((1, 3, input_size, input_size)).astype(np.float32)
    #engine = build_engine(model_path)
    #context = engine.create_execution_context()
    
    in_cpu, out_cpu, in_gpu, out_gpu, stream = alloc_buf(engine)
    res = inference(engine, context, final_img, out_cpu, in_gpu, out_gpu, stream)
    print(res)
    
      
    print("FPS: ", 1/(time.time()-t1))
    
    display.RenderOnce(image, width, height)
    display.SetTitle("{:s} | {:d}x{:d} | {:.0f} FPS".format("Camera Viewer", width, height, display.GetFPS()))
	

# tensorrt docker image: docker pull nvcr.io/nvidia/tensorrt:19.09-py3 (See: https://ngc.nvidia.com/catalog/containers/nvidia:tensorrt/tags)
# NOTE: cuda driver >= 418
