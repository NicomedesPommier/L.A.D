from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import os, argparse

class CORS(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", os.environ.get("CORS_ALLOW_ORIGIN","*"))
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Range, Content-Type")
        super().end_headers()
    def do_OPTIONS(self): self.send_response(204); self.end_headers()

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--dir", default=".")
    p.add_argument("--port", type=int, default=7000)
    args = p.parse_args()
    os.chdir(args.dir)
    ThreadingHTTPServer(("", args.port), CORS).serve_forever()
