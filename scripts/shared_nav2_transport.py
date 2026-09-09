"""Serialized Nav2 action transport shared by fixed-route consumers.

The transport deliberately owns only action send/result/cancel lifecycle.  It
does not plan paths, select goals, publish velocity, or alter Nav2 behavior.
"""


class NavigationActionTransport:
    """Keep exactly one NavigateToPose action alive through its terminal result."""

    def __init__(self, client):
        self._client = client
        self._response_pending = False
        self._handle = None
        self._result = None
        self._cancel_requested = False

    @property
    def response_pending(self):
        return self._response_pending

    @property
    def handle(self):
        return self._handle

    @property
    def result(self):
        return self._result

    @property
    def terminal(self):
        return self._result is not None

    @property
    def busy(self):
        return self._response_pending or self._handle is not None

    def send(self, goal):
        if self.busy:
            raise RuntimeError('NavigateToPose action is not terminal')
        self._response_pending = True
        self._result = None
        self._cancel_requested = False
        self._client.send_goal_async(goal).add_done_callback(self._response)

    def cancel(self):
        """Request cancellation, retaining the slot until the result arrives."""
        if not self.busy or self._cancel_requested:
            return
        self._cancel_requested = True
        if self._handle is not None:
            self._handle.cancel_goal_async()

    def _response(self, future):
        self._response_pending = False
        try:
            handle = future.result()
        except Exception:
            self._result = False
            return
        if handle is None or not handle.accepted:
            self._result = False
            return
        self._handle = handle
        if self._cancel_requested:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self._completed)

    def _completed(self, future):
        try:
            wrapped = future.result()
            self._result = wrapped.status
        except Exception:
            self._result = False
        self._handle = None
