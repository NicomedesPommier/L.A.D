from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import os, argparse

# http_cors.py (reemplaza end_headers)
class CORS(SimpleHTTPRequestHandler):
    def end_headers(self):
        allow = os.environ.get("CORS_ALLOW_ORIGIN", "*")
        origin = self.headers.get("Origin")
        if allow == "*" or not origin:
            self.send_header("Access-Control-Allow-Origin", "*" if allow == "*" else allow.split(",")[0])
        else:
            allowed = [o.strip() for o in allow.split(",")]
            if origin in allowed:
                self.send_header("Access-Control-Allow-Origin", origin)
            else:
                # origen no permitido: no envíes cabecera para que el navegador lo bloquee (más seguro)
                pass
        self.send_header("Vary", "Origin")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Range, Content-Type")
        super().end_headers()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--dir", default=".")
    p.add_argument("--port", type=int, default=7000)
    args = p.parse_args()
    os.chdir(args.dir)
    ThreadingHTTPServer(("", args.port), CORS).serve_forever()
