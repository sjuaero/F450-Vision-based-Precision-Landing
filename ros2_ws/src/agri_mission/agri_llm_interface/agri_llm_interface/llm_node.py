#!/usr/bin/env python3
"""
LLM Interface Node - Parses natural language mission commands using Claude API.

Subscribes to /mission/command_raw (String) and publishes parsed ZoneInfo
to /mission/command_parsed. Also provides /llm/parse_command service.
"""

import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from agri_interfaces.msg import ZoneInfo
from agri_interfaces.srv import ParseMissionCommand

try:
    import anthropic
    ANTHROPIC_AVAILABLE = True
except ImportError:
    ANTHROPIC_AVAILABLE = False


SYSTEM_PROMPT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'prompts',
    'system_prompt.txt'
)


class LLMInterfaceNode(Node):
    def __init__(self):
        super().__init__('llm_interface')

        # Parameters
        self.declare_parameter('model', 'claude-opus-4-6')
        self.declare_parameter('max_tokens', 512)
        self.declare_parameter('api_key_env', 'ANTHROPIC_API_KEY')

        self.model = self.get_parameter('model').value
        self.max_tokens = self.get_parameter('max_tokens').value
        api_key_env = self.get_parameter('api_key_env').value

        # Load system prompt
        try:
            with open(SYSTEM_PROMPT_PATH, 'r', encoding='utf-8') as f:
                self.system_prompt = f.read()
            self.get_logger().info(f'Loaded system prompt from {SYSTEM_PROMPT_PATH}')
        except FileNotFoundError:
            self.get_logger().error(f'System prompt not found at {SYSTEM_PROMPT_PATH}')
            self.system_prompt = ''

        # Initialize Anthropic client
        api_key = os.environ.get(api_key_env)
        if ANTHROPIC_AVAILABLE and api_key:
            self.client = anthropic.Anthropic(api_key=api_key)
            self.get_logger().info(f'Anthropic client initialized with model: {self.model}')
        else:
            self.client = None
            if not ANTHROPIC_AVAILABLE:
                self.get_logger().warn('anthropic package not installed. Using mock mode.')
            else:
                self.get_logger().warn(
                    f'No API key found in env var {api_key_env}. Using mock mode.')

        # Publishers
        self.result_pub = self.create_publisher(ZoneInfo, '/mission/command_parsed', 10)

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/mission/command_raw', self.command_callback, 10)

        # Services
        self.parse_service = self.create_service(
            ParseMissionCommand, '/llm/parse_command', self.parse_service_callback)

        self.get_logger().info('LLM Interface Node started. Waiting for commands...')

    def parse_command(self, text: str) -> dict:
        """Call Claude API to parse command text into structured dict."""
        if self.client is None:
            return self._mock_parse(text)

        try:
            response = self.client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                system=self.system_prompt,
                messages=[{'role': 'user', 'content': text}]
            )
            raw = response.content[0].text.strip()
            # Strip markdown code blocks if present
            if raw.startswith('```'):
                raw = raw.split('```')[1]
                if raw.startswith('json'):
                    raw = raw[4:]
            result = json.loads(raw)
            self.get_logger().info(
                f'LLM parsed: action={result.get("action")}, zone={result.get("zone_id")}, '
                f'confidence={result.get("confidence")}'
            )
            return result
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse LLM response as JSON: {e}')
            return self._error_result(f'JSON parse error: {e}')
        except Exception as e:
            self.get_logger().error(f'LLM API call failed: {e}')
            return self._error_result(str(e))

    def _mock_parse(self, text: str) -> dict:
        """Simple keyword-based fallback when API is unavailable."""
        text_lower = text.lower()
        zone = 'NONE'
        for z in ['A', 'B', 'C', 'D']:
            if z.lower() in text_lower or f'{z}구역' in text or f'zone {z.lower()}' in text_lower:
                zone = z
                break

        if '중단' in text or 'abort' in text_lower or 'stop' in text_lower:
            action = 'abort'
        elif '돌아' in text or 'return' in text_lower or 'base' in text_lower:
            action = 'return_to_base'
        elif zone != 'NONE':
            action = 'spray'
        else:
            action = 'unknown'

        return {
            'action': action,
            'zone_id': zone,
            'confidence': 0.7,
            'language_detected': 'ko' if any('\u3131' <= c <= '\u9fff' for c in text) else 'en',
            'explanation': f'Mock parse: {action} zone={zone}',
            'spray_height_m': 3.0,
            'overlap_percent': 20.0,
            'speed_ms': 2.0,
        }

    def _error_result(self, reason: str) -> dict:
        return {
            'action': 'unknown',
            'zone_id': 'NONE',
            'confidence': 0.0,
            'language_detected': 'unknown',
            'explanation': f'Error: {reason}',
            'spray_height_m': 3.0,
            'overlap_percent': 20.0,
            'speed_ms': 2.0,
        }

    def command_callback(self, msg: String):
        """Process raw command string and publish parsed ZoneInfo."""
        self.get_logger().info(f'Received command: "{msg.data}"')
        result = self.parse_command(msg.data)
        zone_info = self._dict_to_zone_info(result)
        self.result_pub.publish(zone_info)

    def parse_service_callback(self, request, response):
        """Service handler for on-demand command parsing."""
        result = self.parse_command(request.command_text)
        response.success = result.get('action', 'unknown') != 'unknown'
        response.zone_id = result.get('zone_id', 'NONE')
        response.action = result.get('action', 'unknown')
        response.confidence = float(result.get('confidence', 0.0))
        response.explanation = result.get('explanation', '')
        response.spray_height_m = float(result.get('spray_height_m', 3.0))
        response.overlap_percent = float(result.get('overlap_percent', 20.0))
        response.speed_ms = float(result.get('speed_ms', 2.0))
        response.error_message = '' if response.success else result.get('explanation', 'Unknown error')
        return response

    def _dict_to_zone_info(self, d: dict) -> ZoneInfo:
        msg = ZoneInfo()
        msg.zone_id = d.get('zone_id', 'NONE')
        msg.action = d.get('action', 'unknown')
        msg.confidence = float(d.get('confidence', 0.0))
        msg.language_detected = d.get('language_detected', 'unknown')
        msg.explanation = d.get('explanation', '')
        msg.spray_height_m = float(d.get('spray_height_m', 3.0))
        msg.overlap_percent = float(d.get('overlap_percent', 20.0))
        msg.speed_ms = float(d.get('speed_ms', 2.0))
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = LLMInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
