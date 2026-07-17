;; speak.l
;; ROS 2 port of pr2eus/speak.l for voicevox.

(ros::roseus-add-msgs "sound_play_msgs")

(defparameter *voicevox-sound-play-publishers*
  (make-hash-table :test #'equal))

(defun voicevox--arg-to-string (arg)
  (cond
   ((null arg) "")
   ((keywordp arg) (string-downcase (string arg)))
   (t (string arg))))

(defun voicevox--advertise-sound-play-topic (topic-name)
  (unless (gethash topic-name *voicevox-sound-play-publishers*)
    (ros::advertise topic-name sound_play_msgs::SoundRequest 5)
    (setf (gethash topic-name *voicevox-sound-play-publishers*) t)
    (unix:sleep 1)))

(defun play-sound (sound &key arg2 (topic-name "robotsound") wait (volume 1.0))
  "Plays sound using sound_play server.
   Args:
     sound: if sound is pathname, plays sound file located at given path
            if it is number, server plays builtin sound
            otherwise server plays sound as speech sentence
     arg2: voice name or package name, depending on sound_play request type
     topic-name: topic name of sound_play server
     wait: accepted for pr2eus compatibility; topic transport cannot wait for completion
     volume: Volume at which to play the sound. 0 is mute, 1.0 is 100%."
  (let ((msg (instance sound_play_msgs::SoundRequest :init
                       :command sound_play_msgs::SoundRequest::*PLAY_ONCE*
                       :volume volume)))
    (cond
     ((numberp sound)
      (send msg :sound sound))
     ((pathnamep sound)
      (send msg :sound sound_play_msgs::SoundRequest::*PLAY_FILE*)
      (send msg :arg (namestring sound)))
     (t
      (send msg :sound sound_play_msgs::SoundRequest::*SAY*)
      (send msg :arg (string sound))))
    (send msg :arg2 (voicevox--arg-to-string arg2))

    (voicevox--advertise-sound-play-topic topic-name)
    (ros::publish topic-name msg)
    (when wait
      (ros::spin-once)
      (unix:usleep 100000))
    t))

(defun speak (text &key (lang "") (topic-name "robotsound") wait (volume 1.0))
  "Speak sentence using sound_play text-to-speech.
   For VOICEVOX, lang is used as the voice name or speaker id."
  (play-sound text
              :topic-name topic-name
              :wait wait
              :volume volume
              :arg2 lang))

(defun speak-en (text &key (topic-name "robotsound") wait (volume 1.0))
  "Speak english sentence."
  (speak text :topic-name topic-name :wait wait :volume volume))

(defun speak-jp (text &key (topic-name "robotsound_jp") wait (volume 1.0))
  "Speak japanese sentence."
  (speak text :lang :ja :topic-name topic-name :wait wait :volume volume))

(provide :voicevox-speak)
