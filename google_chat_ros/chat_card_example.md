<!-- TODO this should be removed  -->

## Upload img

## Text Message
```yaml
goal:
  text: 'Hello!'
  space: 'AAAAyiXag-0'
```

## Card Message
### Card Message with image
```yaml
goal:
  cards:
    -
      sections:
        -
          widgets:
            -
              image:
                localpath: '/tmp/tweet_image_server.png'
              buttons:
                -
                  text_button_name: 'Manage my supervisor HERE!'
                  text_button_on_click:
                    open_link_url: 'http://supervisor.fetch1075.jsk.imi.i.u-tokyo.ac.jp'
  space: 'AAAAyiXag-0'
```

```yaml
goal:
  cards:
    -
      sections:
        -
          widgets:
            -
              image:
                image_url: 'https://media-cdn.tripadvisor.com/media/photo-s/11/fb/90/e4/dsc-7314-largejpg.jpg'
  space: 'AAAAyiXag-0'
```

### Card Message with header
```yaml
goal:
  cards: 
    -
      header:
        title: 'What do you want to eat?'
        subtitle: 'Executed by Yoshiki Obinata'
        image_localpath: '/tmp/tweet_image_server.png'
  space: 'AAAAyiXag-0'
```

### Text Paragraph
```yaml
goal:
  cards:
    -
      sections:
        -
          widgets: 
            -
              text_paragraph: 'Write a lot of code, paper.'
  space: 'AAAAyiXag-0'
```
```yaml
goal:
  text: 'Hey <users/103866924487978823908>,'
  cards:
    -
      sections:
        -
          widgets: 
            -
              text_paragraph: 'Write <b>a lot of</b> <font color=\"#ff0000\">code</font>, <font color=\"#ff0000\">paper</font>. '
  space: 'AAAAyiXag-0'
```

### KeyValue
```yaml
goal:
  text: 'Something FATAL errors have happened in my computer, please fix ASAP'
  cards:
    -
      sections:
        -
          widgets:
            -
              key_value:
                top_label: 'Process ID'
                content: '1234'
                bottom_label: 'rospy.Exception...'
                on_click:
                  open_link_url: 'http://supervisor.fetch1075.jsk.imi.i.u-tokyo.ac.jp'
                icon: 'DESCRIPTION'
                button:
                  text_button_name: 'Rerun my supervisor HERE!'
                  text_button_on_click:
                    open_link_url: 'http://supervisor.fetch1075.jsk.imi.i.u-tokyo.ac.jp'
  space: 'AAAAyiXag-0'
```

### Order Bot
```yaml
goal:
  cards:
    -
      header:
        title: 'Food delivery errand demo'
        subtitle: 'Executed by Yoshiki Obinata'
        image_url: 'https://inton-shinjukunishiguchi.com/wp-content/uploads/2021/03/uber-eats-logo-1-300x300.png'
      sections:
        -
          widgets:
            -
              key_value:
                top_label: 'Shop'
                content: 'Starbucks'
            - 
              key_value:
                top_label: 'Status'
                content: 'In delivery'
        -
          header: 'Shop info'
          widgets:
            -
              image:
                image_url: 'https://media-cdn.tripadvisor.com/media/photo-s/11/fb/90/e4/dsc-7314-largejpg.jpg'
        -
          widgets:
            -
              buttons:
                -
                  text_button_name: 'ORDER INFO'
                  text_button_on_click:
                    open_link_url: 'https://webapp.starbucks.co.jp/'
  space: 'AAAAyiXag-0'
```

### Interactive button
```yaml
goal:
  cards:
    -
      header:
        title: 'What do you want to eat?'
        subtitle: 'Please choose the food shop!'
      sections:
        -
          widgets:
            -
              buttons:
                -
                  text_button_name: 'STARBUCKS'
                  text_button_on_click:
                    action:
                      action_method_name: 'vote_starbucks'
                      parameters:
                        -
                          key: 'shop'
                          value: 'starbucks'
                -
                  text_button_name: 'SUBWAY'
                  text_button_on_click:
                    action:
                      action_method_name: 'vote_subway'
                      parameters:
                        -
                          key: 'shop'
                          value: 'subway'

  space: 'AAAAyiXag-0'
```
