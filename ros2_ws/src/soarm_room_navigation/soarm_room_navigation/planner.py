"""Configured OpenAI-compatible planner with a bounded, strict JSON boundary."""
import json
import math
import os
import socket
import urllib.error
import urllib.parse
import urllib.request

from .core import NavigationError


MAX_RESPONSE_BYTES = 32768
MAX_JSON_DEPTH = 16


def strict_json_loads(raw):
    def duplicates(items):
        output = {}
        for key, value in items:
            if key in output:
                raise ValueError('duplicate JSON object key')
            output[key] = value
        return output

    def invalid_constant(value):
        raise ValueError('non-finite JSON constant: ' + value)

    value = json.loads(raw, object_pairs_hook=duplicates, parse_constant=invalid_constant)

    def bounded(item, depth=0):
        if depth > MAX_JSON_DEPTH:
            raise ValueError('JSON is nested too deeply')
        if isinstance(item, float) and not math.isfinite(item):
            raise ValueError('non-finite JSON number')
        if isinstance(item, dict):
            if len(item) > 64:
                raise ValueError('JSON object is too large')
            for child in item.values():
                bounded(child, depth + 1)
        elif isinstance(item, list):
            if len(item) > 64:
                raise ValueError('JSON array is too large')
            for child in item:
                bounded(child, depth + 1)
        elif isinstance(item, str) and len(item) > 4096:
            raise ValueError('JSON string is too large')
    bounded(value)
    return value


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, message, headers, newurl):
        raise urllib.error.HTTPError(request.full_url, code, 'redirect rejected', headers, fp)


class OpenAICompatiblePlanner:
    def __init__(self, base_url=None, model=None, api_key=None, timeout=8.0):
        self.base_url = (base_url if base_url is not None else os.environ.get('SOARM_NAV_LLM_BASE_URL', '')).rstrip('/')
        self.model = model if model is not None else os.environ.get('SOARM_NAV_LLM_MODEL', '')
        self.api_key = api_key if api_key is not None else os.environ.get('SOARM_NAV_LLM_API_KEY', '')
        self.timeout = timeout

    @property
    def available(self):
        return bool(self._endpoint_valid() and self.model and self.api_key)

    def _endpoint_valid(self):
        parsed = urllib.parse.urlsplit(self.base_url)
        return bool(parsed.scheme in ('http', 'https') and parsed.netloc and not parsed.username and
                    not parsed.password and not parsed.query and not parsed.fragment)

    @property
    def status(self):
        return 'CONFIGURED' if self.available else 'UNAVAILABLE: set valid SOARM_NAV_LLM_BASE_URL, SOARM_NAV_LLM_MODEL, SOARM_NAV_LLM_API_KEY'

    def plan(self, text, rooms):
        if not self.available:
            raise NavigationError('PLANNER_UNAVAILABLE', self.status)
        if not isinstance(text, str) or not text.strip() or len(text) > 500:
            raise NavigationError('INVALID_INPUT', 'instruction must be 1-500 characters')
        room_data = [{'id': room.id, 'aliases': list(room.aliases)} for room in rooms]
        schema = ('Return one strict JSON object only. Fields are either clarification string, or summary string and actions. '
                  'At most 8 actions. Actions only: {"action":"navigate","room_id":known id,"fallback_room_id":optional known id}, '
                  '{"action":"wait","seconds":1..60}, {"action":"return_to_start","fallback_room_id":optional known id}. '
                  'Do not invent coordinates, tools, loops, unknown fields, or non-JSON text.')
        payload = {'model': self.model, 'temperature': 0, 'max_tokens': 600,
                   'response_format': {'type': 'json_object'},
                   'messages': [{'role': 'system', 'content': schema},
                                {'role': 'user', 'content': json.dumps({'instruction': text, 'rooms': room_data}, ensure_ascii=False)}]}
        request = urllib.request.Request(self.base_url + '/chat/completions', data=json.dumps(payload).encode('utf-8'),
                                         headers={'Content-Type': 'application/json', 'Authorization': 'Bearer ' + self.api_key}, method='POST')
        try:
            opener = urllib.request.build_opener(_NoRedirect())
            with opener.open(request, timeout=self.timeout) as response:
                if response.status != 200:
                    raise NavigationError('PLANNER_FAILURE', 'planner returned HTTP ' + str(response.status))
                raw = response.read(MAX_RESPONSE_BYTES + 1)
                if len(raw) > MAX_RESPONSE_BYTES:
                    raise ValueError('planner response exceeds bound')
                body = strict_json_loads(raw.decode('utf-8'))
            content = body['choices'][0]['message']['content']
            if not isinstance(content, str) or len(content.encode('utf-8')) > MAX_RESPONSE_BYTES:
                raise ValueError('invalid content')
            return strict_json_loads(content)
        except NavigationError:
            raise
        except (urllib.error.URLError, urllib.error.HTTPError, socket.timeout, TimeoutError) as error:
            raise NavigationError('PLANNER_FAILURE', 'configured planner request failed') from error
        except (KeyError, TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError) as error:
            raise NavigationError('PLANNER_SCHEMA', 'configured planner did not return strict JSON plan') from error


class FixturePlanner:
    """Explicit test/demo-only adapter. Never selected by ambient environment."""
    available = True
    status = 'FIXTURE DEMO ONLY - no live LLM'

    def plan(self, text, rooms):
        matches = [room for room in rooms if text.strip().removeprefix('去').strip() in room.aliases]
        if len(matches) != 1:
            return {'clarification': '请指定一个已定义且唯一的房间名称。'}
        return {'summary': '演示计划：前往 ' + matches[0].aliases[0],
                'actions': [{'action': 'navigate', 'room_id': matches[0].id}]}
