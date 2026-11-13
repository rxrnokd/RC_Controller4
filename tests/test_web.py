"""Pi/Linux HTTP + WebSocket integration against disabled real-hardware service."""
import asyncio
import json
from pathlib import Path
import sys
import tempfile
import unittest

try:
    from aiohttp import web
    from aiohttp.test_utils import TestClient, TestServer
except ImportError:
    web = None

from server import Service


@unittest.skipUnless(web and sys.platform == 'linux', 'requires Linux Unix sockets and aiohttp')
class WebTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(__file__).resolve().parents[1]
        c = json.loads((root/'config.json').read_text())
        c['ros']['socket'] = str(Path(self.tmp.name)/'ros.sock')
        self.service = Service(c, root/'config.json')
        app = web.Application()
        app.router.add_get('/ws', self.service.websocket)
        app.on_startup.append(self.service.start)
        app.on_cleanup.append(self.service.close)
        self.client = TestClient(TestServer(app))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()
        self.tmp.cleanup()

    async def reply(self, ws, message):
        await ws.send_json(dict(v=1, **message))
        while True:
            m = await asyncio.wait_for(ws.receive_json(), 1)
            if m['type'] in ('ack', 'error'):
                return m

    async def test_exclusive_owner_estop_and_disconnect(self):
        a, b = await self.client.ws_connect('/ws'), await self.client.ws_connect('/ws')
        self.assertEqual((await self.reply(a, {'type': 'claim', 'source': 'MANUAL'}))['type'], 'ack')
        self.assertEqual((await self.reply(b, {'type': 'claim', 'source': 'MANUAL'}))['type'], 'error')
        self.assertEqual((await self.reply(b, {'type': 'estop'}))['type'], 'ack')
        await a.close()
        await asyncio.sleep(.05)
        self.assertIsNone(self.service.control.owner)
        self.assertFalse(self.service.control.armed)
        await b.close()

    async def test_unconfigured_hardware_never_generates_measurements_or_arms(self):
        ws = await self.client.ws_connect('/ws')
        await self.reply(ws, {'type': 'claim', 'source': 'MANUAL'})
        await self.reply(ws, {'type': 'reset_estop'})
        self.assertEqual((await self.reply(ws, {'type': 'arm'}))['type'], 'error')
        state = self.service.state
        self.assertIsNone(state['odom'])
        self.assertFalse(state['ros']['tracking'])
        self.assertTrue(all(m['rpm'] is None for m in state['sensors']['motors']))
        await ws.close()
