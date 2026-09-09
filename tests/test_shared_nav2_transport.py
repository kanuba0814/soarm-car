import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from shared_nav2_transport import NavigationActionTransport


class Future:
    def __init__(self):
        self.callbacks = []
        self.value = None

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def resolve(self, value):
        self.value = value
        for callback in list(self.callbacks):
            callback(self)

    def result(self):
        return self.value


class Handle:
    accepted = True

    def __init__(self):
        self.cancelled = False
        self.result_future = Future()

    def cancel_goal_async(self):
        self.cancelled = True
        return Future()

    def get_result_async(self):
        return self.result_future


class Client:
    def __init__(self):
        self.sent = []
        self.response = Future()

    def send_goal_async(self, goal):
        self.sent.append(goal)
        return self.response


class SharedNav2TransportTests(unittest.TestCase):
    def test_late_accepted_cancel_remains_busy_until_terminal_result(self):
        client = Client()
        transport = NavigationActionTransport(client)
        transport.send('goal')
        transport.cancel()
        self.assertTrue(transport.busy)
        handle = Handle()
        client.response.resolve(handle)
        self.assertTrue(handle.cancelled)
        self.assertTrue(transport.busy)
        with self.assertRaisesRegex(RuntimeError, 'not terminal'):
            transport.send('replacement')
        handle.result_future.resolve(type('Result', (), {'status': 5})())
        self.assertTrue(transport.terminal)
        self.assertFalse(transport.busy)
        self.assertEqual(transport.result, 5)

    def test_rejected_goal_is_terminal_and_allows_next_send(self):
        client = Client()
        transport = NavigationActionTransport(client)
        transport.send('first')
        client.response.resolve(type('Rejected', (), {'accepted': False})())
        self.assertTrue(transport.terminal)
        self.assertFalse(transport.busy)
        transport.send('second')
        self.assertEqual(client.sent, ['first', 'second'])


if __name__ == '__main__':
    unittest.main()
