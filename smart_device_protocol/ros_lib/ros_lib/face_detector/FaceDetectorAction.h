#ifndef _ROS_face_detector_FaceDetectorAction_h
#define _ROS_face_detector_FaceDetectorAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "face_detector/FaceDetectorActionGoal.h"
#include "face_detector/FaceDetectorActionResult.h"
#include "face_detector/FaceDetectorActionFeedback.h"

namespace face_detector
{

  class FaceDetectorAction : public ros::Msg
  {
    public:
      typedef face_detector::FaceDetectorActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef face_detector::FaceDetectorActionResult _action_result_type;
      _action_result_type action_result;
      typedef face_detector::FaceDetectorActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    FaceDetectorAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "face_detector/FaceDetectorAction"; };
    virtual const char * getMD5() override { return "665c888633df000242196f7098a55805"; };

  };

}
#endif
