import numpy as np
from Brain.src.hardware.realsenseCamera.threads.postProcessing import Postprocessing
from hailo_platform import (HEF, Device, VDevice, HailoStreamInterface, 
    InferVStreams, ConfigureParams, InputVStreamParams, OutputVStreamParams, FormatType)

class HailoInference(Postprocessing):
    
    def __init__(self, hefPath):
        
        device = Device.scan()
        self.hef = HEF(hefPath)
        self.output = self.hef.get_output_vstream_infos()

        self.target = VDevice(device_ids = device)
        self.target.__enter__()
        self.__deviceConfig__

        self.inferPipeline = InferVStreams(self.networkGroup, self.inputVStreamParams, self.outputVStreamParams)
        self.inferPipeline.__enter__()
        
        self.modelArgs = {
            'anchors': {
                'strides': [32, 16, 8], 
                'regression_length': 15
            },
            'classes': 14,
            'img_dims': (640, 640),
            'nms_iou_thresh': 0.2,
            'labels': [
                {'car': 0.3},
                {'crosswalk': 0.35},
                {'hw-entry': 0.35},
                {'hw-exit': 0.35},
                {'lane': 0.1},
                {'no-entry': 0.35},
                {'obstacle': 0.3},
                {'oneway': 0.35},
                {'parking': 0.35},
                {'parking-spot': 0.3},
                {'priority': 0.35},
                {'roundabout': 0.35},
                {'stop-line': 0.3},
                {'stop-sign': 0.35}
            ]
        }
        

        # Adjusted for confidence per class instead of general conf for all classes
        score_threshold = np.empty(self.modelArgs["classes"])
        for labelIdx, label in enumerate(self.modelArgs["labels"]):
            conf_thres = list(label.values())[0]
            assert 0 <= conf_thres <= 1, f"Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0"
            score_threshold[labelIdx] = conf_thres
        self.modelArgs.update({"score_threshold": score_threshold})
            
        assert 0 <= self.modelArgs['nms_iou_thresh'] <= 1, f"Invalid IoU threshold {0 <= self.modelArgs['nms_iou_thresh']}, valid values are between 0.0 and 1.0"
        
        # Remove a specifed grid by index (0: 80x80, 1: 40x40, 2: 20x20)
        self.layersIdxRemove = [0]
        for layerIdx in sorted(self.layersIdxRemove, reverse = False):
            self.modelArgs['anchors']['strides'].pop(2 - layerIdx)
            
            
        super().__init__(self.modelArgs, self.layersIdxRemove)
        
    @property    
    def __deviceConfig__(self):
        
        configureParams = ConfigureParams.create_from_hef(
            self.hef, interface = HailoStreamInterface.PCIe
        )
        self.networkGroup = self.target.configure(self.hef, configureParams)[0]
        self.networkGroupParams = self.networkGroup.create_params()
        self.inputVStreamInfo = self.hef.get_input_vstream_infos()[0]
        self.height, self.width, _ = self.inputVStreamInfo.shape

        self.inputVStreamParams = InputVStreamParams.make_from_network_group(
            self.networkGroup, quantized = False, format_type = FormatType.FLOAT32
        )
        self.outputVStreamParams = OutputVStreamParams.make_from_network_group(
            self.networkGroup, quantized = False, format_type = FormatType.FLOAT32
        )
    
    @property
    def stop(self):
        # Remember to close the damn device
        # Close infer pipeline then the device
        self.inferPipeline.__exit__(*[None] * 3)
        print("\033[92m[INFO]\033[0m Hailo inference pipeline destroyed") 
        self.target.__exit__(*[None] * 3)
        print("\033[92m[INFO]\033[0m Hailo device closed")
        
    
    def inference(self, frame):
        
        inputData = {
            self.inputVStreamInfo.name: frame[None, :, :, :].astype(np.float32) # This thing can run on batch
        }

        with self.networkGroup.activate(self.networkGroupParams):
            rawDetection = self.inferPipeline.infer(inputData)
            
        
        outputDict = {}
        for outLayer in self.output:
            outputDict.update({outLayer.name: rawDetection[outLayer.name]})
        
        return self.postProc(outputDict)