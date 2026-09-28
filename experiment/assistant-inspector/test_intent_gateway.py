import unittest
from intent_gateway import parse_navigation_intent

class IntentGatewayTest(unittest.TestCase):
    def test_text_and_voice_transcript_share_safe_request(self):
        text = parse_navigation_intent("请导航到 room 2")
        voice = parse_navigation_intent("去 room-2", "voice_transcript")
        self.assertEqual(text["request"], voice["request"])
        self.assertEqual(text["status"], "awaiting_confirmation")
        self.assertFalse(text["motion_authorized"])

    def test_unsafe_and_ambiguous_inputs_rejected(self):
        self.assertEqual(parse_navigation_intent("探索一下")["status"], "rejected")
        self.assertEqual(parse_navigation_intent("去 room-1 和 room-2")["reason"], "destination_missing_or_ambiguous")
        self.assertEqual(parse_navigation_intent("去厨房")["status"], "rejected")

if __name__ == "__main__":
    unittest.main()
