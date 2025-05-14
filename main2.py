# Run static server
import static
from gevent.pywsgi import WSGIServer

http_server = WSGIServer(('0.0.0.0', 80), static.app)
http_server.serve_forever()