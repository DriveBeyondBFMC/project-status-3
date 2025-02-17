import numpy as np
import cv2
import line_profiler

profiler = line_profiler.LineProfiler()

class Postprocessing:
    def __init__(self, modelArgs: dict, layersToRemove: list):
        self.modelArgs = modelArgs 
        self.layersToRemove = layersToRemove

    def postProc(self, raw_detections):
    
        raw_detections_keys = list(raw_detections.keys())
        layer_from_shape: dict = {raw_detections[key].shape:key for key in raw_detections_keys}
        
        mask_channels = 32
        
        detection_output_channels = (self.modelArgs['anchors']['regression_length'] + 1) * 4 # (regression length + 1) * num_coordinates
        
    
        endnodes = [raw_detections[layer_from_shape[1, 80, 80, detection_output_channels]],
                    raw_detections[layer_from_shape[1, 80, 80, self.modelArgs['classes']]],
                    raw_detections[layer_from_shape[1, 80, 80, mask_channels]],
                    raw_detections[layer_from_shape[1, 40, 40, detection_output_channels]],
                    raw_detections[layer_from_shape[1, 40, 40, self.modelArgs['classes']]],
                    raw_detections[layer_from_shape[1, 40, 40, mask_channels]],
                    raw_detections[layer_from_shape[1, 20, 20, detection_output_channels]],
                    raw_detections[layer_from_shape[1, 20, 20, self.modelArgs['classes']]],
                    raw_detections[layer_from_shape[1, 20, 20, mask_channels]],
                    raw_detections[layer_from_shape[1, 160, 160, mask_channels]]]

        # Remove a specific mask resolution
        for layerIdx in sorted(self.layersToRemove, reverse = True):
            endnodes.pop(layerIdx * 3)
            endnodes.pop(layerIdx * 3)
            endnodes.pop(layerIdx * 3)
            
            
        predictions_dict = self.segmentPreproc(endnodes, **self.modelArgs)
    
        return predictions_dict

    
    def segmentPreproc(self, endnodes, **modelArgs):
        """
        endnodes is a list of 10 tensors:
            endnodes[0]:  bbox output with shapes (BS, 20, 20, 64)
            endnodes[1]:  scores output with shapes (BS, 20, 20, 80)
            endnodes[2]:  mask coeff output with shapes (BS, 20, 20, 32)
            endnodes[3]:  bbox output with shapes (BS, 40, 40, 64)
            endnodes[4]:  scores output with shapes (BS, 40, 40, 80)
            endnodes[5]:  mask coeff output with shapes (BS, 40, 40, 32)
            endnodes[6]:  bbox output with shapes (BS, 80, 80, 64)
            endnodes[7]:  scores output with shapes (BS, 80, 80, 80)
            endnodes[8]:  mask coeff output with shapes (BS, 80, 80, 32)
            endnodes[9]:  mask protos with shape (BS, 160, 160, 32)
        Returns:
            A list of per image detections, where each is a dictionary with the following structure:
            {
                'detection_boxes':   numpy.ndarray with shape (num_detections, 4),
                'mask':              numpy.ndarray with shape (num_detections, 160, 160),
                'detection_classes': numpy.ndarray with shape (num_detections, 80),
                'detection_scores':  numpy.ndarray with shape (num_detections, 80)
            }
        """
        num_classes = modelArgs["classes"]
        proto_data = endnodes[-1]
        endnodes.pop(-1)
        strides = modelArgs["anchors"]["strides"][::-1]
        image_dims = tuple(modelArgs["img_dims"])
        reg_max = modelArgs["anchors"]["regression_length"]
        raw_boxes = endnodes[::3]
        scores = [np.reshape(s, (-1, s.shape[1] * s.shape[2], num_classes)) for s in endnodes[1::3]]
        scores = np.concatenate(scores, axis=1)
        decoded_boxes = self._yolov8_decoding(raw_boxes, strides, image_dims, reg_max)
        score_thres = modelArgs["score_threshold"]
        iou_thres = modelArgs["nms_iou_thresh"]
        _, self.protoHeight, self.protoWidth, n_masks = proto_data.shape

        # add objectness=1 for working with yolov8_nms
        fake_objectness = np.ones((scores.shape[0], scores.shape[1], 1))
        scores_obj = np.concatenate([fake_objectness, scores], axis=-1)

        coeffs = [np.reshape(c, (-1, c.shape[1] * c.shape[2], n_masks)) for c in endnodes[2::3]]
        coeffs = np.concatenate(coeffs, axis=1)

        # re-arrange predictions for yolov8_nms
        predictions = np.concatenate([decoded_boxes, scores_obj, coeffs], axis=2)

        nms_res = self.non_max_suppression(predictions, conf_thres=score_thres, iou_thres=iou_thres, multi_label=True)

        # No detections occured
        if len(nms_res) == 0:
            return None 

        masks = []
        protos = proto_data[0]
        masks = self.process_mask(protos, nms_res[0]["mask"], nms_res[0]["detection_boxes"], image_dims, nms_res[0]["detection_classes"], upsample = False)
        # print(masks.shape, np.unique(masks))
        output = {}
        output["detection_boxes"] = np.array(nms_res[0]["detection_boxes"]) / np.tile(image_dims, 2)
        if masks is not None:
            output["mask"] = np.transpose(masks, (0, 1, 2))
        else:
            output["mask"] = masks
        output["detection_scores"] = np.array(nms_res[0]["detection_scores"])
        output["detection_classes"] = np.array(nms_res[0]["detection_classes"]).astype(int)
        return output
    
    
    def _yolov8_decoding(self, raw_boxes, strides, image_dims, reg_max):
        boxes = None

        for box_distribute, stride in zip(raw_boxes, strides):
            # create grid
            shape = [int(x / stride) for x in image_dims]
            grid_x = np.arange(shape[1]) + 0.5
            grid_y = np.arange(shape[0]) + 0.5
            grid_x, grid_y = np.meshgrid(grid_x, grid_y)
            ct_row = grid_y.flatten() * stride
            ct_col = grid_x.flatten() * stride
            center = np.stack((ct_col, ct_row, ct_col, ct_row), axis=1)

            # box distribution to distance
            reg_range = np.arange(reg_max + 1)
            box_distribute = np.reshape(
                box_distribute, (-1, box_distribute.shape[1] * box_distribute.shape[2], 4, reg_max + 1)
            )
            box_distance = self._softmax(box_distribute)
            box_distance = box_distance * np.reshape(reg_range, (1, 1, 1, -1))
            box_distance = np.sum(box_distance, axis=-1)
            box_distance = box_distance * stride

            # decode box
            box_distance = np.concatenate([box_distance[:, :, :2] * (-1), box_distance[:, :, 2:]], axis=-1)
            decode_box = np.expand_dims(center, axis=0) + box_distance

            xmin = decode_box[:, :, 0]
            ymin = decode_box[:, :, 1]
            xmax = decode_box[:, :, 2]
            ymax = decode_box[:, :, 3]
            decode_box = np.transpose([xmin, ymin, xmax, ymax], [1, 2, 0])

            xywh_box = np.transpose([(xmin + xmax) / 2, (ymin + ymax) / 2, xmax - xmin, ymax - ymin], [1, 2, 0])
            boxes = xywh_box if boxes is None else np.concatenate([boxes, xywh_box], axis=1)
        
        return boxes  # tf.expand_dims(boxes, axis=2)
    
    @staticmethod
    def crop_mask(masks, boxes):
        """
        Zeroing out mask region outside of the predicted bbox.
        Args:
            masks: numpy array of masks with shape [n, h, w]
            boxes: numpy array of bbox coords with shape [n, 4]
        """

        n_masks, h, w = masks.shape

        # Convert boxes to integer pixel indices
        integer_boxes = np.ceil(boxes).astype(int)

        # Ensure valid indices and avoid negative values
        x1, y1, x2, y2 = np.clip(integer_boxes, 0, [w - 1, h - 1, w - 1, h - 1]).T

        # Create masks for valid regions
        y_grid = np.arange(h).reshape(1, h, 1)  # Broadcastable row indices
        x_grid = np.arange(w).reshape(1, 1, w)  # Broadcastable column indices

        # Vectorized masking
        mask_y = (y_grid >= y1[:, None, None]) & (y_grid <= y2[:, None, None])
        mask_x = (x_grid >= x1[:, None, None]) & (x_grid <= x2[:, None, None])

        # Apply mask
        return np.where(mask_y & mask_x, masks, 0)

    @staticmethod 
    def _sigmoid(x):
        np.negative(x, out=x)  # In-place negation (avoids extra memory allocation)
        np.exp(x, out=x)        # In-place exponentiation
        np.add(x, 1, out=x)     # In-place addition
        np.reciprocal(x, out=x) # In-place division (1/x)
        return x

    @staticmethod
    def _softmax(x):
        x_max = np.max(x, axis=-1, keepdims=True)  # Prevent overflow
        np.subtract(x, x_max, out=x)  # In-place subtraction (avoids memory overhead)
        np.exp(x, out=x)  # In-place exponentiation
        x_sum = np.sum(x, axis=-1, keepdims=True)
        np.divide(x, x_sum, out=x)  # In-place division
        return x  # Reuse modified x

    @staticmethod
    def xywh2xyxy(x):
        y = np.copy(x)
        y[:, 0] = x[:, 0] - x[:, 2] / 2
        y[:, 1] = x[:, 1] - x[:, 3] / 2
        y[:, 2] = x[:, 0] + x[:, 2] / 2
        y[:, 3] = x[:, 1] + x[:, 3] / 2
        return y
        
    
    def non_max_suppression(self, prediction, conf_thres, iou_thres=0.45, max_det=300, nm=32, multi_label=True):
        """Non-Maximum Suppression (NMS) on inference results to reject overlapping detections
        Args:
            prediction: numpy.ndarray with shape (batch_size, num_proposals, 351)
            conf_thres: confidence threshold for NMS
            iou_thres: IoU threshold for NMS
            max_det: Maximal number of detections to keep after NMS
            nm: Number of masks
            multi_label: Consider only best class per proposal or all conf_thresh passing proposals
        Returns:
            A list of per image detections, where each is a dictionary with the following structure:
            {
                'detection_boxes':   numpy.ndarray with shape (num_detections, 4),
                'mask':              numpy.ndarray with shape (num_detections, 32),
                'detection_classes': numpy.ndarray with shape (num_detections, 80),
                'detection_scores':  numpy.ndarray with shape (num_detections, 80)
            }
        """


        nc = prediction.shape[2] - nm - 5  # number of classes

        max_wh = 7680  # (pixels) maximum box width and height
        maskEndIdx = 5 + nc  # mask start index
        output = []
        for image in prediction:  # image index, image inference
            
            # If none remain process next image
            if not image.shape[0]:
                output.append(
                    {
                        "detection_boxes": np.zeros((0, 4)),
                        "mask": np.zeros((0, 32)),
                        "detection_classes": np.zeros((0, 80)),
                        "detection_scores": np.zeros((0, 80)),
                    }
                )
                continue

            # Confidence = Objectness image Class Score
            # image[:, 5:] *= image[:, 4:5] # This remains as original??

            # (center_x, center_y, width, height) to (x1, y1, x2, y2)
            boxes = self.xywh2xyxy(image[:, :4])
            mask = image[:, maskEndIdx:]

            i, j = (image[:, 5:maskEndIdx] > conf_thres).nonzero()
            image = np.concatenate((boxes[i], image[i, 5 + j, None], j[:, None].astype(np.float32), mask[i]), 1)

            # sort by confidence
            image = image[image[:, 4].argsort()[::-1]]

            # per-class NMS
            cls_shift = image[:, 5:6] * max_wh
            boxes = image[:, :4] + cls_shift
            conf = image[:, 4:5]
            preds = np.hstack([boxes.astype(np.float32), conf.astype(np.float32)])

            keep = self.nms(preds, iou_thres)
            if keep.shape[0] > max_det:
                keep = keep[:max_det]
            if keep.shape[0] > 0:
                out = image[keep]
                scores = out[:, 4]
                classes = out[:, 5]
                boxes = out[:, :4]
                masks = out[:, 6:]

                out = {"detection_boxes": boxes, "mask": masks, "detection_classes": classes, "detection_scores": scores}

                output.append(out)

        return output

    @profiler
    def process_mask(self, protos, masks_in, bboxes, shape, classes, upsample=True, downsample=False):
        mh, mw, c = protos.shape
        ih, iw = shape
        maskIdxRemove = np.where(classes == -4)
        maskIdxRetain = np.where(classes != -4)
        maskp = np.delete(masks_in, maskIdxRemove, axis = 0) @ protos.reshape((-1, c)).transpose((1, 0))
        masks = np.zeros((len(classes), mh, mw))
        masks[maskIdxRetain] = self._sigmoid(maskp).reshape((-1, mh, mw))
        

        downsampled_bboxes = bboxes.copy()
        if downsample:
            downsampled_bboxes[:, 0] *= mw / iw
            downsampled_bboxes[:, 2] *= mw / iw
            downsampled_bboxes[:, 3] *= mh / ih
            downsampled_bboxes[:, 1] *= mh / ih

            masks = self.crop_mask(masks, downsampled_bboxes)

        if upsample:
            if not masks.shape[0]:
                return None

            masks = cv2.resize(np.transpose(masks, axes=(1, 2, 0)), shape, interpolation=cv2.INTER_LINEAR)
            if len(masks.shape) == 2:
                masks = masks[..., np.newaxis]
            masks = np.transpose(masks, axes=(2, 0, 1))  # CHW

        if not downsample:
            masks = self.crop_mask(masks, downsampled_bboxes // (self.modelArgs["img_dims"][0] // self.protoHeight) if not upsample else downsampled_bboxes)  # CHW
        
        return masks

    @staticmethod
    def nms(dets: np.ndarray, thresh: float):
        """
        Pure NumPy implementation of Non-Maximum Suppression (NMS).
        
        Args:
            dets (np.ndarray): Array of shape (N, 5), where each row represents
                            [x1, y1, x2, y2, score].
            thresh (float): IoU threshold for suppression.

        Returns:
            np.ndarray: Indices of the selected bounding boxes.
        """
        x1 = dets[:, 0]
        y1 = dets[:, 1]
        x2 = dets[:, 2]
        y2 = dets[:, 3]
        scores = dets[:, 4]

        areas = (x2 - x1 + 1) * (y2 - y1 + 1)  # Compute area of bounding boxes
        order = scores.argsort()[::-1]  # Sort by score in descending order

        suppressed = np.zeros(dets.shape[0], dtype=bool)  # Suppressed mask
        keep = []  # List to store the indices of boxes to keep

        for i in range(len(order)):
            if suppressed[order[i]]:
                continue  # Skip if already suppressed
            
            keep.append(order[i])  # Keep this box

            for j in range(i + 1, len(order)):
                if suppressed[order[j]]:
                    continue

                # Compute IoU (Intersection over Union)
                xx1 = max(x1[order[i]], x1[order[j]])
                yy1 = max(y1[order[i]], y1[order[j]])
                xx2 = min(x2[order[i]], x2[order[j]])
                yy2 = min(y2[order[i]], y2[order[j]])

                w = max(0, xx2 - xx1 + 1)
                h = max(0, yy2 - yy1 + 1)
                inter = w * h
                iou = inter / (areas[order[i]] + areas[order[j]] - inter)

                if iou >= thresh:
                    suppressed[order[j]] = True  # Suppress this box

        return np.array(keep)
    
    def fasterMaskFilter(self, boxCoor, maskPlane, maskThresh):
        maskRatio = self.modelArgs["img_dims"][0] // self.protoHeight
        boxCoor //= maskRatio
        mask = maskPlane[boxCoor[1]: boxCoor[3], boxCoor[0]: boxCoor[2]] > maskThresh 
        maskPlane[:] = 0
        maskPlane[boxCoor[1]: boxCoor[3], boxCoor[0]: boxCoor[2]] = mask
        return maskPlane
    
    @staticmethod
    def drawMask(inputFrame: np.ndarray, maskPlane: np.ndarray):
        # Convert maskPlane to uint8 (needed for OpenCV operations)
        maskPlane = (maskPlane > 0).astype(np.uint8) * 255  # Ensure binary mask

        # Find contours
        contours, _ = cv2.findContours(maskPlane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Overlay setup
        overlay = inputFrame.copy()

        # Fill contours in green (BGR: (0, 255, 0))
        cv2.drawContours(overlay, contours, -1, (0, 255, 0), thickness=cv2.FILLED)

        # Blend with inputFrame using addWeighted
        alpha = 0.5
        cv2.addWeighted(overlay, alpha, inputFrame, 1 - alpha, 0, inputFrame)

    @staticmethod
    def drawBox(inputFrame: np.ndarray, boxCoor: np.ndarray, text: str):
        cv2.putText(inputFrame, f"{text}", (boxCoor[0] + 3, boxCoor[1] - 5), cv2.FONT_HERSHEY_DUPLEX, .5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.rectangle(inputFrame, boxCoor[:2], boxCoor[2:], (0, 0, 255), 2)