#!/usr/bin/env python

import actionlib
from dialogflow_task_executive.msg import RegisterIntentAction, RegisterIntentGoal, RegisterIntentResult, RegisterIntentFeedback
from dialogflow_task_executive.msg import ListIntentAction, ListIntentGoal, ListIntentResult, ListIntentFeedback
from dialogflow_task_executive.msg import IntentInfo
import google.cloud.dialogflow as df
from google.oauth2.service_account import Credentials
import rospy

class DialogflowIntentClient(object):
    def __init__(self):
        credentials_json = rospy.get_param(
            '~google_cloud_credentials_json', None)
        credentials = Credentials.from_service_account_file(credentials_json)
        self.intents_client = df.IntentsClient(credentials=credentials)
        self.project_id = credentials.project_id
        self._intent_create_as = actionlib.SimpleActionServer("~register_intent_action",
                                                              RegisterIntentAction,
                                                              execute_cb=self.register_cb,
                                                              auto_start=False)
        self._intent_list_as = actionlib.SimpleActionServer("~list_intent_action",
                                                            ListIntentAction,
                                                            execute_cb=self.list_cb,
                                                            auto_start=False)
        self._intent_create_as.start()
        self._intent_list_as.start()

    def register_df_intent(self, intent_name, training_phrases, message_texts):
        parent = df.AgentsClient.agent_path(self.project_id)
        phrases = []
        for phrase in training_phrases:
            part = df.Intent.TrainingPhrase.Part(text=phrase)
            training_phrase = df.Intent.TrainingPhrase(parts=[part])
            phrases.append(training_phrase)
        text = df.Intent.Message.Text(text=message_texts)
        message = df.Intent.Message(text=text)
        intent = df.Intent(
            display_name=intent_name, training_phrases=phrases, messages=[message])
        response = self.intents_client.create_intent(
            request={"parent": parent, "intent": intent}
        )
        return response

    def list_df_intent(self):
        parent = df.AgentsClient.agent_path(self.project_id)
        req = df.ListIntentsRequest(parent=parent,
                                    intent_view=df.IntentView.INTENT_VIEW_FULL)
        intents = self.intents_client.list_intents(request=req)
        msgs = []
        for intent in intents:
            msg = IntentInfo()
            msg.intent = intent.display_name
            for training_phrase in intent.training_phrases: # training phrases
                phrase = ""
                for part in training_phrase.parts:
                    phrase += str(part.text)
                msg.concat_training_phrases.append(str(phrase))
            for message in intent.messages:
                msg.message_texts.append(str(message.text))
            msgs.append(msg)
        return msgs

    def register_cb(self, goal):
        feedback = RegisterIntentFeedback()
        result = RegisterIntentResult()
        success = False
        try:
            res = self.register_df_intent(goal.intent.intent,
                                          goal.intent.concat_training_phrases,
                                          goal.intent.message_texts)
            feedback.status = str(res)
            success = True
            rospy.loginfo("Succeeded in registering the intent: {}".format(goal.intent.intent))
        except Exception as e:
            rospy.logerr(str(e))
            feedback.status = str(e)
            success = False
        finally:
            self._intent_create_as.publish_feedback(feedback)
            result.done = success
            self._intent_create_as.set_succeeded(result)

    def list_cb(self, goal):
        feedback = ListIntentFeedback()
        result = ListIntentResult()
        success = False
        try:
            msgs = self.list_df_intent()
            intents_info = ""
            for msg in msgs:
                result.intents.append(msg)
                intents_info += "{}, ".format(msg.intent)
            success = True
            rospy.loginfo("Listed intents: {}".format(intents_info))
            # from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
        except Exception as e:
            rospy.logerr(str(e))
            feedback.status = str(e)
            success = False
        finally:
            self._intent_list_as.publish_feedback(feedback)
            result.done = success
            self._intent_list_as.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("dialogflow_intent_manager")
    node = DialogflowIntentClient()
    rospy.spin()
