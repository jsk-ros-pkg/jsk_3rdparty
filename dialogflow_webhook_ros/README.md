You can recieve the Dialogflow webhook response by `webhook_server.py`.
To use the node, you have to open the port and establish the SSL connection. If you've already opened 443 port but used by another process, you have to use haproxy.
![dialogflow_demo](https://user-images.githubusercontent.com/27789460/146872445-e1ef468a-63fd-4b1a-9eb3-83167e3cb446.gif)

### 1.Get SSL certificate

I recommend to use certbot. Please read the [official guide](https://certbot.eff.org/lets-encrypt/ubuntubionic-other).

### 2.(optional) Enable haproxy and establish forward proxy

If another processes use 443 port, you may use haproxy like [DigitalOceanGuide](https://www.digitalocean.com/community/tutorials/how-to-secure-haproxy-with-let-s-encrypt-on-ubuntu-14-04).

1. Install haproxy

```bash
sudo apt-get update
sudo apt-get install haproxy
```

2. Stop the haproxy service

```bash
sudo systemctl stop haproxy.service
```

3. Copy your pem file to haproxy directory and secure access

```bash
sudo mkdir -p /etc/haproxy/certs
DOMAIN='dialogflow.yourdomain.com' sudo -E bash -c 'cat /etc/letsencrypt/live/$DOMAIN/fullchain.pem /etc/letsencrypt/live/$DOMAIN/privkey.pem > /etc/haproxy/certs/$DOMAIN.pem'
sudo chmod -R go-rwx /etc/haproxy/certs
```

4. Write your `haproxy.cfg` file like [this](https://gist.github.com/mqcmd196/be29f2136b62a7d74d6c3f6c7673b114) and save at `/etc/haproxy/`

5. Run the haproxy service

```bash
sudo systemctl start haproxy.service
```

### 3.setup dialogflow_task_exective webhook_server

Your server's host name, port, certfile and keyfile should be passed to the node. The example `dialogflow_task_executive/config/webhook.json`, is as follows:

```json
{
  "host": "dialogflow.yourdomain.com",
  "port": 8090
  "certfile": "/path/to/your/server.crt"
  "keyfile": "/path/to/your/server.key"
}
```

It is true that `certfile` and `keyfile` are at `/etc/letsencrypt/live/$DOMAIN/`, however non-root users are not permitted to read the file. To avoid it, you exec
```bash
DOMAIN='dialogflow.yourdomain.com' sudo -E bash -c 'cat /etc/letsencrypt/live/$DOMAIN/fullchain.pem > /path/to/your/server.pem'
DOMAIN='dialogflow.yourdomain.com' sudo -E bash -c 'cat /etc/letsencrypt/live/$DOMAIN/privkey.pem > /path/to/your/server.key'
```
To secure the file, you exec
```bash
sudo chmod go-rwx /path/to/your/server.crt
sudo chmod go-rwx /path/to/your/server.key
```

## Author

Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>

## License

BSD
