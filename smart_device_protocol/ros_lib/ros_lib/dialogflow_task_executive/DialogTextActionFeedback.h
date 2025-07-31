#ifndef _ROS_dialogflow_task_executive_DialogTextActionFeedback_h
#define _ROS_dialogflow_task_executive_DialogTextActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "dialogflow_task_executive/DialogTextFeedback.h"

namespace dialogflow_task_executive
{

  class DialogTextActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef dialogflow_task_executive::DialogTextFeedback _feedback_type;
      _feedback_type feedback;

    DialogTextActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "dialogflow_task_executive/DialogTextActionFeedback"; };
    virtual const char * getMD5() override { return "78d9346a781caddd481e52b70b462a2e"; };

  };

}
#endif
