string top_label # The text of the top label
string content # The text of the content. Always required
bool content_multiline # If the content should be multiline
string bottom_label # The text of the bottom label
google_chat_ros/OnClick on_click
string icon # see https://developers.google.com/chat/api/reference/rest/v1/cards#icon
string original_icon_url # Original icon image's url
string original_icon_localpath # If you want to upload new image on google drive, please set this element, not original_icon_url.
google_chat_ros/Button button # A button that can be clicked to trigger an action
