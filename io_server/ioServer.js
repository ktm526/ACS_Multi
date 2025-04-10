/* ──────────────────────────────────────────────────────────
   ioServer.js  ―  Core 서버(3000)와 AMR 사이의 중계 서버
────────────────────────────────────────────────────────── */

const express = require('express');
const http = require('http');
const net = require('net');
const { Server } = require('socket.io');
const multer = require('multer');
const axios = require('axios');
const path = require('path');
const cors = require('cors');
const yaml = require('js-yaml');

const app = express();

/* ───── 환경 상수 ───────────────────────────────────────── */
const AMR_IP = process.env.AMR_IP || '192.168.0.100'; // 기본 AMR IP
const CORE_URL = process.env.CORE_URL || 'http://localhost:3000';
const CMD_PORT = 19304;                                 // 단순 명령 포트
const NAV_PORT = 19206;                                 // Navigation API 포트

const MSG_GOTARGET_ID = 0x0BEB;  // 3051 robot_task_gotarget_req
const MSG_GOTARGET_RES = 0x32FB;  // 13051 robot_task_gotarget_res
let serialCounter = 1;
const MAP_REQ_ID = 0x0514;   // 1300
const MAP_RES_ID = 0x2C24;   // 11300

/* ───── 공통 미들웨어 ──────────────────────────────────── */
app.use(cors());
app.use(express.json({ limit: '500mb' }));
app.use(express.urlencoded({ limit: '500mb', extended: true }));

/* ──────────────────────────────────────────────────────────
   1. 파일 업로드 / 맵 변환 (기존 기능)
────────────────────────────────────────────────────────── */

const storage = multer.memoryStorage();
const fileFilter = (req, file, cb) => {
    const ext = path.extname(file.originalname).toLowerCase();
    if (['.smap', '.json', '.pgm', '.yaml', '.yml'].includes(ext)) cb(null, true);
    else cb(new Error('Only .smap, .json, .pgm, .yaml/.yml files are allowed!'));
};
const upload = multer({ storage, fileFilter });

/* ───── 유틸 함수 ───────────────────────────────────────── */
function euclideanDistance(a, b) {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    return Math.sqrt(dx * dx + dy * dy);
}

function transformMapData(mapJson) {
    /* advancedPointList / advancedCurveList 형식 처리 */
    let stations = [];
    if (Array.isArray(mapJson.advancedPointList)) {
        stations = mapJson.advancedPointList.map(p => ({
            id: p.instanceName,
            x: p.pos.x,
            y: p.pos.y,
            connections: [],
            property: p.property || {}
        }));
    } else throw new Error('advancedPointList is missing');

    const findStation = id => stations.find(st => st.id.toString() === id.toString());
    let paths = [];
    if (Array.isArray(mapJson.advancedCurveList)) {
        mapJson.advancedCurveList.forEach(curve => {
            const parts = curve.instanceName.split('-');
            if (parts.length !== 2) return;
            const [id1, id2] = parts;
            const st1 = findStation(id1);
            const st2 = findStation(id2);
            if (!st1 || !st2) return;
            const dist = euclideanDistance({ x: st1.x, y: st1.y }, { x: st2.x, y: st2.y });
            st1.connections.push({ station: id2, distance: dist, property: curve.property || {} });
            st2.connections.push({ station: id1, distance: dist, property: curve.property || {} });
            paths.push({
                start: id1, end: id2,
                coordinates: { start: { x: st1.x, y: st1.y }, end: { x: st2.x, y: st2.y } },
                property: curve.property || {}
            });
        });
    }
    return {
        name: mapJson.header?.mapName || 'Unnamed Map',
        station: JSON.stringify({ stations }),
        path: JSON.stringify({ paths }),
        additional_info: JSON.stringify({
            header: mapJson.header,
            normalPosList: mapJson.normalPosList,
            advancedLineList: mapJson.advancedLineList
        })
    };
}

function transformMapDataFromNodesEdges(jsonData, yamlData, normalPosList) {
    const station = JSON.stringify({ stations: jsonData.nodes });
    const paths = jsonData.edges.map(edge => {
        const s = jsonData.nodes.find(n => n.id === edge.start);
        const e = jsonData.nodes.find(n => n.id === edge.end);
        if (s && e) {
            return {
                start: edge.start, end: edge.end,
                coordinates: { start: { x: s.x, y: s.y }, end: { x: e.x, y: e.y } }
            };
        }
        return null;
    }).filter(Boolean);
    return {
        name: yamlData.header?.mapName || 'Unnamed Map',
        station, path: JSON.stringify({ paths }),
        additional_info: JSON.stringify({ normalPosList })
    };
}

function parsePGM(buffer) {
    const text = buffer.toString('ascii');
    const lines = text.split('\n').filter(l => l[0] !== '#');
    const magic = lines[0].trim();
    if (!['P5', 'P2'].includes(magic)) throw new Error('Unsupported PGM format');
    const [width, height] = lines[1].trim().split(/\s+/).map(Number);
    let headerLen = 0, cnt = 0;
    for (let i = 0; i < buffer.length; i++) {
        if (buffer[i] === 0x0A) { cnt++; if (cnt === 3) { headerLen = i + 1; break; } }
    }
    const pixelData = magic === 'P5'
        ? buffer.slice(headerLen)
        : text.substring(headerLen).trim().split(/\s+/).map(Number);
    return { width, height, pixelData, isBinary: magic === 'P5' };
}

/* ------------------- /uploadMap ------------------------- */
app.post('/uploadMap', upload.fields([
    { name: 'mapFile', maxCount: 1 },
    { name: 'pgmFile', maxCount: 1 },
    { name: 'yamlFile', maxCount: 1 },
]), async (req, res) => {
    try {
        /* JSON mapFile 처리 */
        if (req.files?.mapFile) {
            const content = req.files.mapFile[0].buffer.toString('utf8');
            let mapJson; try { mapJson = JSON.parse(content); } catch { return res.status(400).json({ success: false, message: 'Invalid JSON' }); }

            /* advancedPointList 방식 */
            if (mapJson.advancedPointList) {
                const data = transformMapData(mapJson);
                const core = await axios.post(`${CORE_URL}/api/maps`, data);
                return res.status(core.data?.success ? 201 : 500).json(core.data);
            }
            /* nodes/edges 방식 */
            if (mapJson.nodes && mapJson.edges) {
                if (!(req.files.pgmFile && req.files.yamlFile))
                    return res.status(400).json({ success: false, message: 'PGM & YAML required' });
                const yamlData = yaml.load(req.files.yamlFile[0].buffer.toString('utf8'));
                const pgm = parsePGM(req.files.pgmFile[0].buffer);
                const normalPosList = [];
                const reso = parseFloat(yamlData.resolution) || 0.03;
                const [ox, oy] = yamlData.origin || [0, 0, 0];
                const { width: w, height: h } = pgm;
                for (let i = 0; i < h; i++) {
                    for (let j = 0; j < w; j++) {
                        const val = pgm.pixelData[j + i * w];
                        if (val === 0) {
                            normalPosList.push({ x: ox + j * reso, y: oy + (h - i - 1) * reso });
                        }
                    }
                }
                const data = transformMapDataFromNodesEdges(mapJson, yamlData, normalPosList);
                const core = await axios.post(`${CORE_URL}/api/maps`, data);
                return res.status(core.data?.success ? 201 : 500).json(core.data);
            }
            return res.status(400).json({ success: false, message: 'Invalid JSON structure' });
        }

        /* PGM+YAML만 업로드 */
        if (req.files?.pgmFile && req.files?.yamlFile) {
            const yamlData = yaml.load(req.files.yamlFile[0].buffer.toString('utf8'));
            const pgm = parsePGM(req.files.pgmFile[0].buffer);
            const normalPosList = [];
            const reso = parseFloat(yamlData.resolution) || 0.03;
            const [ox, oy] = yamlData.origin || [0, 0, 0];
            const { width: w, height: h } = pgm;
            for (let i = 0; i < h; i++) {
                for (let j = 0; j < w; j++) {
                    const val = pgm.pixelData[j + i * w];
                    if (val === 0) normalPosList.push({ x: ox + j * reso, y: oy + (h - i - 1) * reso });
                }
            }
            const data = {
                name: yamlData.header?.mapName || 'Unnamed Map',
                station: "", path: "",
                additional_info: JSON.stringify({ normalPosList })
            };
            const core = await axios.post(`${CORE_URL}/api/maps`, data);
            return res.status(core.data?.success ? 201 : 500).json(core.data);
        }

        return res.status(400).json({ success: false, message: 'Unsupported upload' });
    } catch (e) {
        console.error('Error in /uploadMap:', e);
        return res.status(500).json({ success: false, message: e.message });
    }
});


/* ───────────────────────────────
   2. AMR Push API 클라이언트
───────────────────────────────*/
const PUSH_PORT = 19301;
const robotLastReceived = new Map();

/* ── 헤더 파서 ───────────────── */
function parseHeader(buf) {
    return {
        sync: buf.readUInt8(0),
        ver: buf.readUInt8(1),
        serial: buf.readUInt16BE(2),
        len: buf.readUInt32BE(4),
        type: buf.readUInt16BE(8)
    };
}

/* ── 로봇 한 대와 구독 세션 만들기 ─ */
function subscribePush(amrIp) {
    const socket = new net.Socket();
    let buffer = Buffer.alloc(0);

    socket.setTimeout(10_000);

    socket.connect(PUSH_PORT, amrIp, () =>
        console.log(`[Push] connected to ${amrIp}:${PUSH_PORT}`)
    );

    socket.on("data", async (chunk) => {
        buffer = Buffer.concat([buffer, chunk]);

        while (buffer.length >= 16) {
            if (buffer.readUInt8(0) !== 0x5A) {
                console.warn("[Push] bad sync byte, drop");
                buffer = Buffer.alloc(0);
                break;
            }
            const header = parseHeader(buffer.slice(0, 16));
            if (buffer.length < 16 + header.len) break; // not full packet yet

            const packet = buffer.slice(0, 16 + header.len);
            buffer = buffer.slice(16 + header.len);

            try {
                const json = JSON.parse(packet.slice(16).toString());
                const name =
                    json.vehicle_id || json.robot_id || "UnknownRobot";

                /* 수신 타임스탬프 기록 */
                robotLastReceived.set(name, Date.now());

                /* 위치·상태 DB 업데이트 */
                const pos = {
                    x: json.x ?? json.position?.x ?? 0,
                    y: json.y ?? json.position?.y ?? 0,
                    angle: json.angle ?? json.position?.yaw ?? 0,
                };
                await axios.post(`${CORE_URL}/api/robots`, {
                    name,
                    status: json.status,
                    position: JSON.stringify(pos),
                    additional_info: JSON.stringify(json),
                    timestamp: new Date(),
                    ip: amrIp,
                });
            } catch (e) {
                console.error("[Push] JSON parse error:", e.message);
            }
        }
    });

    socket.on("error", (e) => {
        console.error(`[Push] socket error (${amrIp}):`, e.message);
        retry();
    });

    socket.on("close", () => {
        console.warn(`[Push] connection closed (${amrIp})`);
        retry();
    });

    socket.on("timeout", () => {
        console.warn(`[Push] timeout (${amrIp})`);
        socket.destroy();
    });

    /* 재연결 로직 */
    function retry() {
        setTimeout(() => subscribePush(amrIp), 5_000);
    }
}

/* ── AMR IP 목록(여러 대면 배열) ─ */
const AMR_IPS = (process.env.AMR_IPS || "192.168.0.100")
    .split(",")
    .map((s) => s.trim());

AMR_IPS.forEach(subscribePush);

/* ── “연결 끊김” 감지 타이머 ─ */
setInterval(async () => {
    const now = Date.now();
    for (const [name, last] of robotLastReceived) {
        if (now - last > 5_000) {
            robotLastReceived.delete(name);
            try {
                await axios.post(`${CORE_URL}/api/robots`, {
                    name,
                    status: "연결 끊김",
                    timestamp: new Date(),
                });
            } catch (e) {
                console.error("[Push] disconnect update error:", e.message);
            }
        }
    }
}, 1_000);


/* ──────────────────────────────────────────────────────────
   3. /api/sendTask  ― Core → ioServer
────────────────────────────────────────────────────────── */

function buildPacket(type, json, serial) {
    const body = Buffer.from(JSON.stringify(json), 'utf8');
    const head = Buffer.alloc(16);
    head.writeUInt8(0x5A, 0);
    head.writeUInt8(0x01, 1);
    head.writeUInt16BE(serial, 2);
    head.writeUInt32BE(body.length, 4);
    head.writeUInt16BE(type, 8);
    return Buffer.concat([head, body]);
}
function parseHeader(buf) { return { len: buf.readUInt32BE(4), type: buf.readUInt16BE(8) }; }

app.post('/api/sendTask', async (req, res) => {
    const robot_ip = req.body.robot_ip || AMR_IP;
    const task = req.body.task;
    if (!task) return res.status(400).json({ success: false, message: 'task is required' });
    const robot_name = req.body.robot_name || task.robot_name || null;

    const steps = typeof task.steps === 'string' ? JSON.parse(task.steps) : task.steps;
    const route = steps.filter(s => s.stepType === '경로').map(s => s.description);
    if (!route.length) return res.status(400).json({ success: false, message: 'no route steps' });

    res.json({ success: true });                      // 즉시 OK
    runTaskOverTcp(robot_ip, robot_name, task, route).catch(console.error);
});

async function runTaskOverTcp(ip, robotName, task, route) {
    const socket = net.createConnection({ host: ip, port: NAV_PORT });
    socket.setTimeout(20000);

    let buffer = Buffer.alloc(0), idx = 0;

    const sendDest = () => {
        const dest = route[idx];
        const serial = serialCounter = (serialCounter + 1) & 0xFFFF;
        const payload = {
            id: dest,
            source_id: idx === 0 ? 'SELF_POSITION' : route[idx - 1],
            task_id: task.id?.toString(),
            method: 'forward',
            skill_name: 'GotoSpecifiedPose',
        };
        socket.write(buildPacket(MSG_GOTARGET_ID, payload, serial));
    };
    sendDest();

    socket.on('data', async chunk => {
        buffer = Buffer.concat([buffer, chunk]);
        while (buffer.length >= 16) {
            const h = parseHeader(buffer.slice(0, 16));
            if (buffer.length < 16 + h.len) break;
            const pkt = buffer.slice(0, 16 + h.len);
            buffer = buffer.slice(16 + h.len);
            if (h.type !== MSG_GOTARGET_RES) continue;

            const body = JSON.parse(pkt.slice(16).toString());
            if (body.ret_code !== 0) return handleError('robot ret_code ' + body.ret_code);

            /* 정상 도달 */
            idx++;
            const nextDest = route[idx] || null;
            try {
                if (robotName) {
                    await axios.post(`${CORE_URL}/api/robots`, {
                        name: robotName,
                        destination: nextDest,
                        status: nextDest ? '이동' : '대기',
                        timestamp: new Date(),
                    });
                }
                if (!nextDest) {
                    await axios.put(`${CORE_URL}/api/tasks/${task.id}`, {
                        status: 'completed', updated_at: new Date(),
                    });
                    socket.end(); return;
                }
            } catch (e) { return handleError('core update failed: ' + e.message); }
            sendDest();
        }
    });

    socket.on('error', e => handleError('TCP error: ' + e.message));
    socket.on('timeout', () => handleError('TCP timeout'));

    async function handleError(msg) {
        console.error('[sendTask] ' + msg);
        socket.destroy();
        try {
            const promises = [
                axios.put(`${CORE_URL}/api/tasks/${task.id}`, {
                    status: '오류', updated_at: new Date()
                })
            ];
            if (robotName) {
                promises.unshift(
                    axios.post(`${CORE_URL}/api/robots`, {
                        name: robotName, status: '오류', timestamp: new Date()
                    })
                );
            }
            await Promise.all(promises);
        } catch (e) { console.error('[sendTask] fail to mark error:', e.message); }
    }
}

/* ──────────────────────────────────────────────────────────
   4. (옵션) /ello/taskstart  ― 옛 단순 배열 전송 방식
────────────────────────────────────────────────────────── */
app.post('/ello/taskstart', async (req, res) => {
    const { robot_ip, task } = req.body;
    if (!robot_ip || !task) return res.status(400).json({ success: false, message: 'robot_ip & task required' });
    const steps = typeof task.steps === 'string' ? JSON.parse(task.steps) : task.steps;
    const routeSteps = steps.filter(s => s.stepType === '경로').map(s => Number(s.description));
    const client = net.createConnection({ host: robot_ip, port: CMD_PORT }, () => {
        client.write(JSON.stringify(routeSteps));
    });
    client.on('data', async data => {
        let msg; try { msg = JSON.parse(data.toString()); } catch { client.end(); return; }
        try {
            await axios.post(`${CORE_URL}/api/tasks/updateStatus`, msg);
        } catch (e) { console.error('updateStatus error:', e.message); }
        client.end();
    });
    client.on('error', e => console.error('taskstart TCP error:', e.message));
    return res.json({ success: true, message: 'Task command sent' });
});

app.get('/api/robot/:name/maps', async (req, res) => {
    try {
        /* 1) DB 에서 로봇 IP 가져오기 */
        const robotRes = await axios.get(
            `${CORE_URL}/api/robots`,
            { params: { name: req.params.name } }
        );
        const robot = robotRes.data?.data?.[0];
        if (!robot?.ip) return res.status(404).json({ success: false, message: 'Robot IP not found' });

        /* 2) TCP 요청 → 1300(robot_status_map_req) */
        const client = new net.Socket();
        client.setTimeout(5_000);

        const packet = Buffer.alloc(16);
        packet.writeUInt8(0x5A, 0); // sync
        packet.writeUInt8(0x01, 1); // version
        packet.writeUInt16BE(1, 2); // serial
        packet.writeUInt32BE(0, 4); // body length 0
        packet.writeUInt16BE(MAP_REQ_ID, 8);
        // reserved 6 bytes = 0

        const result = await new Promise((resolve, reject) => {
            let buf = Buffer.alloc(0);
            client.connect(19204, robot.ip, () => client.write(packet));

            client.on('data', (chunk) => {
                buf = Buffer.concat([buf, chunk]);
                if (buf.length >= 16) {
                    const type = buf.readUInt16BE(8);
                    const len = buf.readUInt32BE(4);
                    if (type === MAP_RES_ID && buf.length >= 16 + len) {
                        try {
                            const json = JSON.parse(buf.slice(16, 16 + len).toString());
                            resolve(json);
                        } catch (e) { reject(e); }
                        client.end();
                    }
                }
            });
            client.on('error', reject);
            client.on('timeout', () => reject(new Error('TCP timeout')));
        });

        return res.json({ success: true, data: result });
    } catch (e) {
        console.error('/api/robot/:name/maps error', e.message);
        return res.status(500).json({ success: false, message: e.message });
    }
});


/* ──────────────────────────────────────────────────────────
   5. Socket.IO & HTTP 서버 구동
────────────────────────────────────────────────────────── */
const server = http.createServer(app);
const io = new Server(server, { cors: { origin: '*' } });
io.on('connection', s => {
    console.log('socket.io client', s.id);
    s.on('disconnect', () => console.log('socket.io disconnect', s.id));
});

app.get('/', (_, res) => res.send('ioServer is running on port 4000'));

const PORT = 4000;
server.listen(PORT, () => console.log('ioServer running on', PORT));

module.exports = { app, io };
