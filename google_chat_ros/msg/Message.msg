string name # Resource name in form spaces/*/messages/*
google_chat_ros/User sender # The user who created the message
string create_time # The time at which the message was created in Google Chat server
string text # Plain-text body of the message
string thread_name # The thread the message belongs to
google_chat_ros/Annotation[] annotations # Annotations associated with the plain-text body of the message
string argument_text # Plain-text body of the message with all bot mentions stripped out
google_chat_ros/Attachment[] attachments # User uploaded attachment
