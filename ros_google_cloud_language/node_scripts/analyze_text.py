#!/usr/bin/env python
# -*- coding: utf-8 -*-

# beased on example code
# https://googleapis.dev/python/language/latest/usage.html#documents
# 
# ros code
import actionlib
import rospy
import os

from ros_google_cloud_language.msg import AnalyzeTextAction
from ros_google_cloud_language.msg import AnalyzeTextResult
from ros_google_cloud_language.msg import AnalyzeTextFeedback
from ros_google_cloud_language.msg import TextEntity
from ros_google_cloud_language.msg import TextSyntax
from diagnostic_msgs.msg import KeyValue

# Imports the Google Cloud client library
from google.cloud import language_v1 as language

class ROSGoogleCloudLanguage(object):
    def __init__(self):
        credentials_path = rospy.get_param("~google_cloud_credentials_json", None)
        if credentials_path:
            if not os.path.exists(credentials_path):
                raise Exception("file {} is not found, please specify existing google cloud credentials json file path".format(credentials_path))
            os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = credentials_path
        # Instantiates a client
        self._client = language.LanguageServiceClient()
        #
        self._as = actionlib.SimpleActionServer(
            '~text', AnalyzeTextAction,
            execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        feedback = AnalyzeTextFeedback()
        result = AnalyzeTextResult()
        success = True

        try:
            document = language.types.Document(
                content=goal.text,
                type='PLAIN_TEXT'
            )
            # Detects the entities and sentiment of the text
            response = self._client.analyze_entities(
                 document=document,
                 encoding_type='UTF32',
            )
            for entity in response.entities:
                result.entities.append(TextEntity(
                    name=entity.name.encode('utf-8'),
                    type=entity.type,
                    metadata=map(lambda kv: KeyValue(kv[0], kv[1]),
                                 entity.metadata.items()),
                    salience=entity.salience))

            # Detects the sentiment of the text
            sentiment = self._client.analyze_sentiment(
                document=document,
                encoding_type='UTF32'
            ).document_sentiment

            result.sentiment_score = sentiment.score
            result.sentiment_magnitude = sentiment.magnitude

            # # Analayze Syntax
            syntax = self._client.analyze_syntax(
                document=document,
                encoding_type='UTF32'
            )

            for token in syntax.tokens:
                # print("{} {} {}".format(token.part_of_speech.tag, token.text.content.encode('utf-8'), token.dependency_edge.head_token_index)
                result.syntaxes.append(TextSyntax(
                    name=token.text.content.encode('utf-8'),
                    lemma=token.lemma.encode('utf-8'),
                    dependency_edge=token.dependency_edge.head_token_index,
                    part_of_speech=token.part_of_speech.tag,
                    parse_label=token.dependency_edge.label
                ))

        except Exception as e:
            rospy.logerr("Fail to analyze syntax ... {}".format(str(e)))
            feedback.status = str(e)
            success = False
        finally:
            self._as.publish_feedback(feedback)
            self._as.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node("ros_google_cloud_language")
    lang = ROSGoogleCloudLanguage()
    rospy.spin()
