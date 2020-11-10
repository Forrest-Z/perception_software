#include "lidar_tracking/kalman_tracking/kalman_multiple_tracker.h"
#include "lidar_common/assignment_problem_solver.h"
#include <iostream>

#define INIT_FRAME (2)
#define MERGE_THRESHOLD (2.8f)

namespace perception {
namespace lidar {

KalmanMultipleTracker::KalmanMultipleTracker()
{
    init();
    //std::cout << "KalmanMultipleTracker()" <<std::endl;
}

KalmanMultipleTracker::~KalmanMultipleTracker()
{
    listTrackers.clear();
    //std::cout << "~KalmanMultipleTracker()" <<std::endl;
}

void KalmanMultipleTracker::mutilpleTracking(const double stamp, 
                                             const std::vector<base::ObjectPtr>& detections, 
                                             const int tracker_flag)
{
    std::vector<TrackedObject> trackedObjects;
    trackedObjects.clear();
    if(tracker_flag == 0){
        detectFrontObjectsToTreackedObjects(stamp, detections, trackedObjects);
        this->tracker_flag = tracker_flag;
    }
    else if(tracker_flag == 1){
        detectBehindObjectsToTreackedObjects(stamp, detections, trackedObjects);
        this->tracker_flag = tracker_flag;
    }
    else{
        objectsToTreackedObjects(stamp, detections, trackedObjects);
    }
    mutilpleTracking(stamp, trackedObjects);
    perStamp = stamp;
}

void KalmanMultipleTracker::detectFrontObjectsToTreackedObjects(const double stamp,
                                                                const std::vector<base::ObjectPtr> &detections,
                                                                std::vector<TrackedObject> &trackingObjects)
{
    trackingObjects.clear();
    for(auto object: detections){
        TrackedObject tempObject(object);
        tempObject.timestamp = stamp;
        tempObject.center[0] = tempObject.center[0];
        tempObject.center[1] = tempObject.center[1] - tempObject.size[1] / 2.0f;
        tempObject.center[2] = tempObject.center[2];
        trackingObjects.push_back(tempObject);
    }
}

void KalmanMultipleTracker::detectBehindObjectsToTreackedObjects(const double stamp,
                                                                 const std::vector<base::ObjectPtr> &detections,
                                                                 std::vector<TrackedObject> &trackingObjects)
{
    trackingObjects.clear();
    for(auto object: detections){
        TrackedObject tempObject(object);
        tempObject.timestamp = stamp;
        tempObject.center[0] = tempObject.center[0];
        tempObject.center[1] = tempObject.center[1] + tempObject.size[1] / 2.0f;
        tempObject.center[2] = tempObject.center[2];
        trackingObjects.push_back(tempObject);
    }
}

void KalmanMultipleTracker::objectsToTreackedObjects(const double stamp,
                                                     const std::vector<base::ObjectPtr> &detections,
                                                     std::vector<TrackedObject> &trackingObjects)
{
    trackingObjects.clear();
    for(auto object: detections){
        TrackedObject tempObject(object);
        tempObject.timestamp = stamp;
        trackingObjects.push_back(tempObject);
    }
}

void KalmanMultipleTracker::mutilpleTracking(const double stamp, const std::vector<TrackedObject>& detections)
{
    double dt = stamp - perStamp;
    //std::cout << "dt:" << dt << std::endl;
    if(dt > 0.5)
    {
        std::cerr << "lidar tracker dt:" << dt << std::endl;
        dt = 0.1;
    }
    else if(dt < 0.0000001)
    {
        std::cerr << "lidar tracker dt:" << dt << std::endl;
        dt = 0.1;
    }
    if (detections.size() <= 0)
    {
        for (size_t i=0; i<listTrackers.size(); i++)
        {
            listTrackers[i]->addSkippedFrame(1);
            if (listTrackers[i]->getSkippedFrame() > maximum_allowed_skipped_frames)
            {
                listTrackers.erase(listTrackers.begin() + i);
                i--;
            }
            else
            {
                listTrackers[i]->setDeltatime(dt);
                listTrackers[i]->kalmanPrediction();
                if (listTrackers[i]->getTraceSize() > max_trace_length)
                {
                    listTrackers[i]->eraseTrace(max_trace_length);
                }
            }
        }
        perStamp = stamp;
        return;
    }

    // If there is no tracks yet, then every point begins its own track.
    if (listTrackers.size() == 0)
    {
        // If no tracks yet,every point setup tracker
        for (size_t i = 0; i< detections.size(); i++)
        {
            nextTrackerID++;
            std::shared_ptr<KalmanTracker> tracker(new KalmanTracker(nextTrackerID));
            tracker->addTrace(detections[i]);
            listTrackers.push_back(tracker);
        }
    }
    else
    {
        matchAndTrackingObject(dt, detections);
    }

    for(size_t i = 0; i< listTrackers.size(); i++)
    {
        if(!listTrackers[i]->isTracking() && listTrackers[i]->getTraceSize() == INIT_FRAME)
        {
            listTrackers[i]->initKalman();
        }
    }

    mergeObject.mergeAll(listTrackers, MERGE_THRESHOLD);
    perStamp = stamp;
}

void KalmanMultipleTracker::getTrackedObjectList(std::vector<base::ObjectPtr> &result)
{
    result.clear();
    for(auto &tracker: listTrackers)
    {
        if(tracker->isTracking()){
            TrackedObject tracked_object = tracker->getPredictObject();
            base::ObjectPtr object(new base::Object(*tracked_object.object_ptr));
            object->track_id = static_cast<int>(tracked_object.tracker_id);
            object->center = tracked_object.center.cast<double>();
            object->size = tracked_object.size;
            object->velocity = tracked_object.velocity;
            object->theta = tracked_object.theta;
            object->direction = tracked_object.direction;
            switch(this->tracker_flag){
                case 0:
                    object->center[1] = object->center[1] + object->size[1] / 2.0;
                    result.push_back(object);
                    break;
                case 1:
                    object->center[1] = object->center[1] - object->size[1] / 2.0;
                    result.push_back(object);
                    break;
                default:
                    result.push_back(object);
                    break;
            }
        }
    }
}

int KalmanMultipleTracker::getTrackersCount()
{
    return static_cast<int>(this->listTrackers.size());
}

void KalmanMultipleTracker::matchAndTrackingObject(const double dt, const std::vector<TrackedObject>& detections)
{
    size_t N = listTrackers.size();
    size_t M = detections.size();
    std::vector< std::vector<double> > Cost(N, std::vector<double>(M));
    std::vector<int> assignment;
    // Solving assignment problem (listTrackers and predictions of Kalman filter)
    AssignmentProblemSolver APS;

    for (size_t i=0; i<listTrackers.size(); i++)
    {
        listTrackers[i]->setDeltatime(dt);
        listTrackers[i]->kalmanPrediction();
    }

    calculateEuclideanDistance(detections, Cost);

    APS.Solve(Cost, assignment);

    // clean assignment from pairs with large distance
    for (size_t i = 0; i< assignment.size(); i++)
    {
        if (assignment[i] != -1)
        {
            if (Cost[i][assignment[i]] > dist_thres)
            {
                assignment[i] = -1;
            }
        }
    }

    // Search for unassigned detects
    std::vector<int> notAssignedDetections;
    std::vector<int>::iterator it;
    for (size_t i = 0; i < detections.size(); i++)
    {
        it = std::find(assignment.begin(), assignment.end(), i);
        if (it == assignment.end())
        {
            notAssignedDetections.push_back(i);
        }
    }

    // and start new trackers for them.
    if (notAssignedDetections.size() != 0)
    {
        for (size_t i = 0; i < notAssignedDetections.size(); i++)
        {
            nextTrackerID++;
            std::shared_ptr<KalmanTracker> tracker(new KalmanTracker(nextTrackerID));
            tracker->addTrace(detections[notAssignedDetections[i]]);
            listTrackers.push_back(tracker);
        }
    }

    // Update Kalman Filters state
    for (size_t i = 0; i< assignment.size(); i++)
    {
        if(assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
        {
            listTrackers[i]->setSkippedFrame(0);
            listTrackers[i]->kalmanUpdate(detections[assignment[i]]);
        }
        if(listTrackers[i]->isTracking())
        {
            if(listTrackers[i]->getTraceSize() > max_trace_length)
            {
                listTrackers[i]->eraseTrace(max_trace_length);
            }
        }
        else
        {
            if(listTrackers[i]->getTraceSize() > INIT_FRAME)
            {
                listTrackers[i]->eraseTrace(INIT_FRAME);
            }
        }
    }

    for (size_t i = 0; i< assignment.size(); i++)
    {
        if (assignment[i] == -1)
        {
            // If tracker have no assigned detect, then increment skipped frames counter.
            listTrackers[i]->addSkippedFrame(1);
            //If tracker didn't get detects long time, remove it.
            if (listTrackers[i]->getSkippedFrame() > maximum_allowed_skipped_frames)
            {
                listTrackers.erase(listTrackers.begin() + i);
                assignment.erase(assignment.begin() + i);
                i--;
            }
        }
    }
}

void KalmanMultipleTracker::calculateEuclideanDistance(const std::vector<TrackedObject>& detections,
                                std::vector< std::vector<double> > &costMatrix)
{
    float dist = 0;
    for (size_t i = 0; i<listTrackers.size(); i++)
    {
        for (size_t j = 0; j<detections.size(); j++)
        {
            cv::Point2f point1(listTrackers[i]->getPredictObject().center[0],
                    listTrackers[i]->getPredictObject().center[1]);
            cv::Point2f point2(detections[j].center[0], detections[j].center[1]);
            cv::Point2f diff = (point1 - point2);
            dist = std::sqrt(diff.x*diff.x + diff.y*diff.y);
            costMatrix[i][j] = static_cast<double>(dist);
        }
    }
}

void KalmanMultipleTracker::init()
{
    nextTrackerID = 0;
    isFirstRun = true;
    listTrackers.clear();
    loadConfig();
    perStamp = 0;
    tracker_flag = -1;
}

void KalmanMultipleTracker::saveConfig()
{
    cv::FileStorage fs;
    fs.open("./KalmanMultipleTracker.xml", cv::FileStorage::WRITE);

    cv::write(fs, "dist_thres", dist_thres);
    cv::write(fs, "maximum_allowed_skipped_frames", maximum_allowed_skipped_frames);
    cv::write(fs, "max_trace_length", max_trace_length);

    fs.release();
}

void KalmanMultipleTracker::loadConfig()
{
    cv::FileStorage fs;
    fs.open("./KalmanMultipleTracker.xml", cv::FileStorage::READ);

    cv::read(fs["dist_thres"], dist_thres, 5.0);
    cv::read(fs["maximum_allowed_skipped_frames"], maximum_allowed_skipped_frames, 4);
    cv::read(fs["max_trace_length"], max_trace_length, 10);

    fs.release();
}

}  // namespace lidar
}  // namespace perception