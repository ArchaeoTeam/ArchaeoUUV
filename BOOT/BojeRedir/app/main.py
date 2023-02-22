#!/usr/bin/env python3

import sys
from http.server import HTTPServer, BaseHTTPRequestHandler

class Redirect(BaseHTTPRequestHandler):
   def do_GET(self):
       self.send_response(302)
       self.send_header('Location', "http://192.168.2.3:80")
       self.end_headers()

HTTPServer(("", int(8121)), Redirect).serve_forever()
