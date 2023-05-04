#!/usr/bin/env python
import argparse
import cv2
import numpy as np
import logging
from collections import OrderedDict
import time,sys
from matplotlib import pyplot as plt
from common import *
import operator as op
import math

STREAM_FPS = 1 #used for ttc
RATIO = 0.75
AGE = 10

def ClusterKeypoints(keypoints, kphist, img, epsilon=0):
    if len(keypoints) < 2: return []

    cluster = []
    unclusteredKPs = sorted(keypoints,key=op.attrgetter('pt'))
    while unclusteredKPs:
        clust = [unclusteredKPs.pop(0)]
        kp = clust[0]
        i = 0
        while i < len(unclusteredKPs):
            if overlap(kp, unclusteredKPs[i], eps=epsilon):
                clust.append(unclusteredKPs.pop(i))
            else:
                i += 1
        if (len(clust) >= 3): cluster.append(Cluster(clust,img))

    return cluster


logger = logging.getLogger(__name__)
logging.basicConfig()

def mse(img1, img2):
   h, w = img1.shape
   diff = cv2.subtract(img1, img2)
   err = np.sum(diff**2)
   mse = err/(float(h*w))
   return mse
 
parser = argparse.ArgumentParser(usage="moa.py [options]")

#SIFT Params
parser.add_argument("-c", "--contrast-threshold", type=float, default=0.04
                  , help="Set the contrast threshold for SIFT. (default=0.04)")

parser.add_argument("-e", "--edge-threshold", type=float, default=10
                  , help="Set the edge threshold for SIFT. (default=10)")
#Matching Params
parser.add_argument("-d", "--distance", type=float, default=200.0
                  , help="Distance (in pixels?) of each keypoint match. (default=200.0)")

parser.add_argument("-s", "--scale", type=float, default=1.2
                  , help="Scale at which we filter keypoints that are getting larger. (default=1.2)")

parser.add_argument("--roi", type=int, default=4
                  , help="Factor for computing the region of interest (default=4).")

parser.add_argument("-l", "--logging", type=int, default=20,
                    help="Log level (DEBUG=10, INFO=20, WARNING=30, ERROR=40, CRITICAL=50) [default: 20]")

parser.add_argument("--video-file", default=None
                    , help="Load a video file to test.")

parser.add_argument("--sleep",  type=int, default=1000,
                    help="Sleep with waitKey for this many ms (default=1000). Use 0 to wait indefinitely for keypress.")

parser.add_argument("--epsilon", type=int, default=50, help="Maximum inter-distance between cluster points.")

parser.add_argument("--features", type=int, default=0, help="The number of top features to keep; all if set to 0.")

parser.add_argument("--output", default=None
                    , help="Generate specified mp4 file from frames processed.")

parser.add_argument("--knn", action='store_true', help="Use knn matching and ratio test.")
#parser.add_argument("--knn", action='store_true', help="Use knn matching and ratio test.")


opts = parser.parse_args()
logger.setLevel(opts.logging)

cv2.namedWindow("SIFT", flags=cv2.WINDOW_NORMAL)
cv2.namedWindow("MATCHES", flags=cv2.WINDOW_NORMAL)
cv2.namedWindow("OBSTACLES", flags=cv2.WINDOW_NORMAL)
cv2.namedWindow("BLACK_WHITE", flags=cv2.WINDOW_NORMAL)
output_frames = []


capture = cv2.VideoCapture(opts.video_file)
ret, prev_img = capture.read()
while not ret: 
    logger.error(f"Failed to read from VideoCapture: {ret}")
    ret, prev_img = capture.read()
#convert to greyscale
prev_img = cv2.cvtColor(prev_img,cv2.COLOR_BGR2GRAY)

bfmatcher = cv2.BFMatcher(cv2.NORM_L2)
sift = cv2.SIFT_create(nfeatures=opts.features, contrastThreshold=opts.contrast_threshold, edgeThreshold=opts.edge_threshold)
id = 1
# mask out region of interest
try:
    roi = np.zeros(prev_img.shape,np.uint8)
    scrapY, scrapX = prev_img.shape[0]//opts.roi, prev_img.shape[1]//opts.roi
    roi[scrapY:-scrapY, scrapX:-scrapX] = True
except Exception as e:
    logger.error(e)
    

# get keypoints and feature descriptors
prev_kps, prev_descs = sift.detectAndCompute(prev_img,roi)
#logger.info("First point: {}, {}".format(prev_kps[0], prev_descs[0]))
#logger.info("Length of prev kps:{}".format(len(prev_kps)))
for kp in prev_kps: 
    kp.class_id = id
id += 1

feature_img = cv2.drawKeypoints(prev_img, prev_kps, None,(0,0,255),4)
cv2.imshow("SIFT", feature_img)
cv2.waitKey(1)

t_last = time.time()
kpHist = OrderedDict()
while True:
    ret, img = capture.read()
    if not ret:
        break
    t_curr = time.time()
    dispim = img.copy()

    if not ret:
        logger.error(f"Failed to read frame from capture device.")
        break
    else:
        #logger.info(f"Frame #{id}")
        #convert to greyscale
        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # get keypoints and feature descriptors
        kps, descs = sift.detectAndCompute(img,roi)
        for kp in kps: 
            kp.class_id = id
        #print("Current kps:{}".format(len(kps)))
        id += 1
        feature_img = cv2.drawKeypoints(img, kps, None,(0,0,255),4)
        cv2.rectangle(feature_img,(scrapX,scrapY),(feature_img.shape[1]-scrapX,feature_img.shape[0]-scrapY),(0,255,255),thickness=1)
        cv2.imshow("SIFT", feature_img)
        cv2.waitKey(1)

        if prev_descs is not None:
            train_indices_matched = []
            prev_kps_not_matched = []
            prev_descs_not_matched = []
            if opts.knn:
                matches = bfmatcher.knnMatch(descs, prev_descs, k=2)
                # Apply ratio test
                good = []
                for m,n in matches:
                    logger.debug(f"M-dist: {m.distance}, N-dist: {n.distance}")
                    if m.distance < RATIO*n.distance:
                        good.append(m)
                #logger.info(f"Good Matches (passed ratio test of  < {RATIO}): {len(good)}")

            else:
                matches = bfmatcher.match(descs, prev_descs)
                for m in matches:
                    #print(m, m.queryIdx, m.trainIdx)
                    pass
                # Sort them in the order of their distance.
                ## find the list of kps that were not matched.

                matches = sorted(matches, key = lambda x:x.distance)
                #logger.info(f"Total matches: {len(matches)}")

                # Filter out bad matches
                good = []
                for m in matches:
                    logger.debug(f"M-dist: {m.distance}")
                    if m.distance < opts.distance:
                        good.append(m)
                        train_indices_matched.append(m.trainIdx)
                #print("Length of train indices matched:", len(train_indices_matched))
                for each_prev_kp_index in range(len(prev_kps)):
                    if  each_prev_kp_index not in train_indices_matched:
                        prev_kps_not_matched.append(prev_kps[each_prev_kp_index])
                        prev_descs_not_matched.append(prev_descs[each_prev_kp_index])
                        #print(each_prev_kp_index)

                if(len(matches) > 0):
                    max_value = max(matches, key=lambda x : x.distance)
                    min_value = min(matches, key=lambda x : x.distance)
                    #logger.info(f"Min/Max dist: {min_value.distance} / {max_value.distance}")
                #logger.info(f"Good Matches (distance < than {opts.distance}): {len(good)}")
        
        # Compare sizes of kps and eliminate all that are the same or getting smaller.
        ###########################################################################
        ##Change this part here.
        bigger = []
        expandingKPs = []
        for g in good:
            #logger.info(f"Key point: {kps[g.queryIdx].pt}, {kps[g.queryIdx].size}")
            #logger.debug(f"Prev Size: {prev_kps[g.trainIdx].size}, Current Size: {kps[g.queryIdx].size}")
            clsid = prev_kps[g.trainIdx].class_id

            curr_point_x, curr_point_y = int(np.round(kps[g.queryIdx].pt[0])), int(np.round(kps[g.queryIdx].pt[1]))
            prev_point_x, prev_point_y = int(np.round(prev_kps[g.trainIdx].pt[0])), int(np.round(prev_kps[g.trainIdx].pt[1]))

            size_expansion = 1.5 ### MAKE THIS TUNABLE
            curr_size = int(max(1, np.round((kps[g.queryIdx].size)*size_expansion)))
            prev_size = int(max(1, np.round((prev_kps[g.trainIdx].size)*size_expansion)))
            if prev_size <= curr_size - 2:
                pass#logger.info(f"Prev size and curr size is: {prev_size}, {curr_size}")
            curr_total_x, curr_total_y = img.shape[1], img.shape[0]
            prev_total_x, prev_total_y = prev_img.shape[1], prev_img.shape[0]
            if (curr_point_x + curr_size/2 + 1 > curr_total_x) or (curr_point_y + curr_size/2 + 1 > curr_total_y) or (prev_point_x + prev_size/2 + 1 > prev_total_x) or (prev_point_y + prev_size/2 + 1 > prev_total_y) or \
                    (curr_point_x - curr_size / 2 - 1 < 0) or (curr_point_y - curr_size / 2 - 1 < 0) or (prev_point_x - prev_size / 2 - 1 < 0) or (prev_point_y - prev_size / 2 - 1 < 0):
                continue
            if curr_size < prev_size + 2: continue


            # extract sub image from perev image
            ## new image borders
            if curr_size %2 == 1:
                left = int(curr_point_x - math.floor(curr_size/2))
                right = int(curr_point_x + math.ceil(curr_size/2))
                top = int(curr_point_y - math.floor(curr_size/2))
                bottom = int(curr_point_y + math.ceil(curr_size/2))
            else:
                left = int(curr_point_x - curr_size / 2)
                right = int(curr_point_x + curr_size / 2)
                top = int(curr_point_y - curr_size / 2)
                bottom = int(curr_point_y + curr_size / 2)

            temp_curr_image = np.asarray(img[top:bottom, left:right])
            #logger.info(f"Temp currimage shape:{temp_curr_image.shape}")

            if prev_size % 2 == 1:
                left = int(prev_point_x - math.floor(prev_size / 2))
                right = int(prev_point_x + math.ceil(prev_size / 2))
                top = int(prev_point_y - math.floor(prev_size / 2))
                bottom = int(prev_point_y + math.ceil(prev_size / 2))
            else:
                left = int(prev_point_x - prev_size / 2)
                right = int(prev_point_x + prev_size / 2)
                top = int(prev_point_y - prev_size / 2)
                bottom = int(prev_point_y + prev_size / 2)

            temp_prev_image = np.asarray(prev_img[top:bottom, left:right])
            #logger.info(f"Temp prev image shape:{temp_prev_image.shape}")

            #logger.info("Shapes:{}, {}".format(temp_curr_image.shape, temp_prev_image.shape))

            ## loop from previous key point length to current key point length
            scale_results=np.empty(0)
            results_dict = {}
            for expansion in range(0, curr_size - prev_size, 2):
                #logger.info("Nums:{}, {}, {}, {}".format(expansion, expansion//2, curr_size - expansion//2, curr_size))
                cropped_temp_curr_image = temp_curr_image[expansion//2:curr_size - expansion//2, expansion//2:curr_size - expansion//2]
                resized_temp_prev_image = cv2.resize(temp_prev_image, (curr_size - expansion, curr_size - expansion))
                #logger.info("Cropped Shapes:{}, {}".format(cropped_temp_curr_image.shape, resized_temp_prev_image.shape))

                error = mse(cropped_temp_curr_image, resized_temp_prev_image)
                results_dict[error] = resized_temp_prev_image.shape[0]/temp_prev_image.shape[0]
            best = min(results_dict.keys())
            ratio = results_dict[best]
            #logger.info("Ratio:{}".format(ratio))


            if(ratio > opts.scale): ## changed the condition check for clustering
            ################################################################
            #if(kps[g.queryIdx].size > opts.scale*prev_kps[g.trainIdx].size):
                bigger.append(g)
                expandingKPs.append(prev_kps[g.trainIdx])
                clsid = prev_kps[g.trainIdx].class_id
                if clsid not in kpHist:
                    kpHist[clsid] = KeyPointHistory()
                    t_A = t_last
                else:
                    t_A = kpHist[clsid].timehist[-1][-1]

                kpHist[clsid].update(prev_kps[g.trainIdx], prev_descs[g.trainIdx], t_A, t_curr, kps[g.queryIdx].size)
        
        #logger.info(f"Bigger Matches (curr.size > {opts.scale} * prev.size): {len(bigger)}")
        matches_img = cv2.drawMatches(img,kps,prev_img,prev_kps,bigger,None,(128,128,0),(0,0,255),flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS|cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow("MATCHES", matches_img)

        kpHist = OrderedDict([kv for kv in iter(kpHist.items()) if kv[1].downdate().age < AGE])

        #missed = [k for k in iter(kpHist.keys()) if (kpHist[k].age > 0) and (k not in bigger)]
        #missed = [(kpHist[k].keypoint, kpHist[clsid].descriptor.reshape(1,-1)) for k in missed]
        #if missed:
        #    missed_kp, missed_desc = list(zip(*missed))
        #    prev_kps = prev_kps + missed_kp
        #    prev_descs = missed_desc[0] if prev_descs is None else np.r_[prev_descs, missed_desc[0]]

        # cluster keypoints and sort my maximum inter-cluster distance
        cluster = ClusterKeypoints(expandingKPs, kpHist, img, epsilon=opts.epsilon)

        # Draw clusters with tags
        votes = [sum(kpHist[kp.class_id].detects for kp in c.KPs) for c in cluster]
        b_w_disp = dispim.copy()
        b_w_disp[0:b_w_disp.shape[0], 0:dispim.shape[1]] = (255, 255, 255)
        for c in cluster:
            clustinfo = "%d" % len(c.KPs)
            logger.info("COORDS: {}, {}, {}, {} ".format(c.p0[1],c.p1[1], c.p0[0],c.p1[0]))
            # Draw an arrow denoting the direction to avoid obstacle
            x_obs, y = c.pt
            if (x_obs-(img.shape[1]//2)) < 0: offset = 50
            if x_obs >= (img.shape[1]//2):    offset = -50
            cv2.arrowedLine(dispim, (img.shape[1]//2, img.shape[0] - 50)
                            , (img.shape[1]//2+offset, img.shape[0] - 50)
                            , (0,255,0), 3)

            # draw cluster ranking
            clr = (0,255-sum(kpHist[kp.class_id].detects for kp in c.KPs)*165./max(votes),255)
            cv2.putText(dispim, clustinfo, (c.p1[0]-(c.p1[0]-c.p0[0])//2, c.p1[1]-(c.p1[1]-c.p0[1])//2)
                        ,cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.3, color=clr, thickness=1)

            cv2.rectangle(dispim, c.p0, c.p1, color=clr,thickness=0)
            #print("Points:", c.p0, c.p1)

            #b_w_disp[c.p0[1]:c.p1[1], c.p0[0]:c.p1[0]] = (0,0,0)
            b_w_disp[0:img.shape[1], c.p0[0]:c.p1[0]] = (0,0,0)

        b_w_disp = b_w_disp[ scrapY : b_w_disp.shape[0] - scrapY,scrapX : b_w_disp.shape[1] - scrapX]
        b_w_disp = cv2.cvtColor(b_w_disp, cv2.COLOR_BGR2GRAY)
        #draw roi
        cv2.rectangle(dispim, (scrapX,scrapY), (dispim.shape[1]-scrapX, dispim.shape[0]-scrapY), (0,255,255), thickness=1)
        # convert the grayscale image to binary image
        #logger.info(b_w_disp)
        #ret, thresh = cv2.threshold(b_w_disp, 254, 255, 0)
        #logger.info("THRESHOLD:{}".format(thresh))

        # find contours in the binary image
        contours, h = cv2.findContours(b_w_disp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #cv2.drawContours(dispim, contours, -1, (0, 255, 0), 3)
        #logger.info(contours)
        try:
            c = max(contours, key=cv2.contourArea)
            # calculate moments for each contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            #logger.info("CX, CY {}, {}".format(cX, cY))
            cv2.circle(dispim, (scrapX + cX, scrapY + cY), 5, (0, 255, 0), -1)
            cv2.putText(dispim, "safe", (scrapX + cX, scrapY + cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        except:
            pass

        cv2.imshow("OBSTACLES", dispim)
        cv2.imshow("BLACK_WHITE", b_w_disp)


        if opts.output:
            height, width, layers = dispim.shape
            size = (width,height)
            output_frames.append(dispim)

        cv2.waitKey(opts.sleep)
        prev_img = img

        kps = list(kps)
        prev_kps = kps + prev_kps_not_matched
        prev_kps = tuple(prev_kps)

        #print(type(prev_descs))
        prev_descs = np.vstack((descs, prev_descs_not_matched))
        '''prev_descs = list(prev_descs)
        prev_descs = descs + prev_descs_not_matched
        prev_descs = tuple(prev_descs)
        prev_kps = kps #+ prev_kps_not_matched
        prev_descs = descs'''
        t_last = t_curr



if opts.output:
    out = cv2.VideoWriter(opts.output,cv2.VideoWriter_fourcc(*'mp4v'), 10, size)
    for i in range(len(output_frames)):
        out.write(output_frames[i])
    out.release()

capture.release()
cv2.destroyAllWindows()
