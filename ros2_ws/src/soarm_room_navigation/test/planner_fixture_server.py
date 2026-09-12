"""Local protocol fixture ONLY: no language understanding or real model inference."""
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_POST(self):
        size = int(self.headers.get('Content-Length', '0'))
        if self.path != '/v1/chat/completions' or not 0 < size <= 65536:
            self.send_error(400)
            return
        body = json.loads(self.rfile.read(size))
        instruction = json.loads(body['messages'][-1]['content'])['instruction']
        if instruction == 'fixture:roundtrip':
            actions = [{'action': 'navigate', 'room_id': 'qa_room'},
                       {'action': 'wait', 'seconds': 1},
                       {'action': 'return_to_start'}]
        elif instruction == 'fixture:cancel':
            actions = [{'action': 'navigate', 'room_id': 'qa_room'}]
        else:
            self.send_error(400, 'Unknown fixture scenario')
            return
        plan = {'summary': '协议测试夹具，不是真实语言模型', 'actions': actions}
        data = json.dumps({'choices': [{'message': {'content': json.dumps(plan)}}]}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)
        print(json.dumps({'fixture': instruction, 'action_count': len(actions)}), flush=True)

if __name__ == '__main__':
    port = int(os.environ.get('SOARM_NAV_QA_MODEL_PORT', '18192'))
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    print(f'PROTOCOL_FIXTURE_READY 127.0.0.1:{port} (not an LLM)', flush=True)
    server.serve_forever()
