# MCP Robot Server - MVP

FastAPI server exposing robot control (arm + hand) via HTTP API and MCP protocol.

## Quick Start

### 1. Start Hardware Bridge (WSL host)
```bash
cd /home/zhang123/ros2_ws/lerobotTest
python bridge.py --host 0.0.0.0 --port 9000
```

### 2. Start MCP Server (Docker)
```bash
cd mcp_server
docker-compose up --build
```

### 3. Test
```bash
# HTTP API
curl http://localhost:8000/health
curl http://localhost:8000/api/v1/hand/status
curl -X POST http://localhost:8000/api/v1/hand/angles \
  -H "Content-Type: application/json" \
  -d '{"angles": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]}'

# MCP Protocol
curl http://localhost:8000/mcp/tools/list
curl -X POST http://localhost:8000/mcp/tools/call \
  -H "Content-Type: application/json" \
  -d '{"name": "hand_set_angles", "arguments": {"angles": [0, 0, 0, 0, 0, 0]}}'
```

## Architecture

```
Claude Desktop → MCP Server (Docker) → Hardware Bridge (WSL) → Real Hardware
```

## Endpoints

### HTTP API
- `GET /health` - Health check
- `GET /api/v1/hand/status` - Hand status
- `POST /api/v1/hand/angles` - Set hand angles
- `POST /api/v1/hand/gesture/{name}` - Execute gesture
- `GET /api/v1/arm/status` - Arm status (TODO)

### MCP Protocol
- `POST /mcp/tools/list` - List available tools
- `POST /mcp/tools/call` - Call a tool
