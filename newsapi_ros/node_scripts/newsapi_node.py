#!/usr/bin/env python

import actionlib
from newsapi import NewsApiClient
from newsapi.newsapi_client import NewsAPIException
from newsapi_ros.msg import GetTopHeadlinesAction, GetTopHeadlinesFeedback, GetTopHeadlinesResult, News
import rospy

class NewsApiNode(object):

    CATEGORIES = ["business", "entertainment", "general", "health", "science", "sports", "technology"]

    def __init__(self):
        self.language = rospy.get_param("~language", None)
        self.country = rospy.get_param("~country", "jp")
        _api_key = rospy.get_param("~api_key")
        self._newsapi = NewsApiClient(api_key=_api_key)
        self._top_headlines_as = actionlib.SimpleActionServer("~get_top_headlines",
                                                              GetTopHeadlinesAction,
                                                              execute_cb=self._top_headlines_cb,
                                                              auto_start=False)
        # TODO add EVERYTHING. c.f. https://newsapi.org/docs/endpoints/everything
        self._top_headlines_as.start()


    def _top_headlines_cb(self, goal):
        feedback = GetTopHeadlinesFeedback()
        result = GetTopHeadlinesResult()
        success = False
        # Request
        keyword = goal.keyword if goal.keyword else None
        sources = ",".join(goal.sources) if ",".join(goal.sources) else None
        if sources:
            rospy.logwarn("Sources param was set. Category and country are ignored")
        # sources param cannot be mixed with the ``country`` or ``category`` params
        if not goal.category in self.CATEGORIES:
            raise KeyError("Invalid categories. Choose from {}".format(",".join(self.CATEGORIES)))
        category = goal.category if (goal.category and not sources) else None
        country = self.country if (not sources) else None
        try:
            res = self._newsapi.get_top_headlines(q=keyword,
                                                  sources=sources,
                                                  category=category,
                                                  language=self.language,
                                                  country=country)
            news_infos = []
            for article in res['articles']:
                 news = News()
                 news.source = str(article.get("source").get("name"))
                 news.author = str(article.get("author"))
                 news.title = str(article.get("title"))
                 news_infos.append(news.title)
                 news.description = str(article.get("description"))
                 news.url = str(article.get("url"))
                 news.url_to_image = str(article.get("urlToImage"))
                 news.published_at = str(article.get("published_at"))
                 news.content = str(article.get("content"))
                 result.news_list.append(news)
            success = True
            rospy.loginfo("Got Top Headlines news. {}".format(", ".join(news_infos)))
        except (NewsAPIException, SyntaxError, ValueError) as e:
            rospy.logerr(e)
            feedback.status = str(e)
            success = False
        finally:
            self._top_headlines_as.publish_feedback(feedback)
            result.done = success
            self._top_headlines_as.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("newsapi_ros")
    node = NewsApiNode()
    rospy.spin()
