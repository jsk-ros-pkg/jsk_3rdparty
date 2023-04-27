import rospy
from std_msgs.msg import String
from deep_translator import (
    GoogleTranslator,
    MicrosoftTranslator,
    PonsTranslator,
    LingueeTranslator,
    MyMemoryTranslator,
    YandexTranslator,
    PapagoTranslator,
    #DeeplTranslator,
    #QcriTranslator,
    single_detection,
    batch_detection)
from dynamic_reconfigure.server import Server
from ros_translator.cfg import ROSTranslatorConfig

engines = {
    'google': GoogleTranslator,
    'microsoft': MicrosoftTranslator,
    'pons': PonsTranslator,
    'linguee': LingueeTranslator,
    'mymemory': MyMemoryTranslator,
    'yandex': YandexTranslator,
    'papago': PapagoTranslator,
    #'deepl': DeeplTranslator,
    #'qcri': QcriTranslator,
}


class ROSTranslator(object):

  def __init__(self):

    self.from_language = rospy.get_param('~from_language', 'auto')
    self.to_language = rospy.get_param('~to_language', 'en')
    self.translator = rospy.get_param('~translator', 'google')
    self.api_key = rospy.get_param('~api_key', '')
    self.engine = None
    self.update_engine()

    self.param_srv = Server(ROSTranslatorConfig, self.param_callback)
    self.pub = rospy.Publisher('~output_text', String, queue_size=1)
    self.sub = rospy.Subscriber('~input_text', String, self.callback)

  def update_engine(self):

    if self.translator in engines:
      if self.translator == 'yandex':
        self.engine = engines[self.translator](self.api_key)
      elif self.translator == 'microsoft':
        self.engine = engines[self.translator](api_key=self.api_key,
                                               target=self.to_language)
      else:
        self.engine = engines[self.translator](source=self.from_language,
                                               target=self.to_language)
    else:
      rospy.logerr('Unknown translator backend: {}'.format(self.translator))
      self.engine = None

  def translate(self, text):

    if self.engine is not None:
      if self.translator == 'yandex':
        return self.engine.translate(source=self.from_language,
                                     target=self.to_language,
                                     text=text)
      elif self.translator == 'microsoft':
        return self.engine.translate(text)
      else:
        return self.engine.translate(text)
    else:
      rospy.logwarn('Engine not loaded.')
      return ''

  def callback(self, msg):

    output_msg = String()
    output_msg.data = self.translate(msg.data)
    self.pub.publish(output_msg)

  def param_callback(self, config, level):

    self.from_language = config['from_language']
    self.to_language = config['to_language']
    self.translator = config['translator']
    self.api_key = config['api_key']
    self.update_engine()
    return config
