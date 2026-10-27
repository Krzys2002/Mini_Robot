/* Deterministic tests of the actual prototype page, runnable with Node or V8. */
async function runBrowserTests(html) {
    const source = html.match(/<script>([\s\S]*?)<\/script>/)[1];
    function assert(value, message) { if (!value) throw new Error(message); }
    function fixture() {
        const elements = new Map(), windowEvents = {}, timers = new Map(), requests = [];
        let releaseMove, blockMove = false, failStop = false;
        const document = { getElementById(id) {
            if (!elements.has(id)) elements.set(id, {
                value: id === 'ip' ? '127.0.0.1' : id === 'distance' ? '0.1' : '0',
                checked: false, style: {}, textContent: '', events: {},
                addEventListener(type, callback) { this.events[type] = callback; },
                getBoundingClientRect() { return {left: 0, top: 0}; }
            });
            return elements.get(id);
        }};
        const window = {addEventListener(type, callback) { windowEvents[type] = callback; }};
        const fetch = async url => {
            const path = url.replace('http://127.0.0.1', ''); requests.push(path);
            if (path.startsWith('/move') && blockMove) {
                blockMove = false; await new Promise(resolve => { releaseMove = resolve; });
            }
            if (path === '/estop' && failStop) return {ok: false, text: async () => 'stop failed'};
            return {ok: true, json: async () => ({ok: true})};
        };
        const app = new Function('document', 'window', 'setInterval', 'fetch', source +
            '\nreturn {sendMove,sendTarget,emergencyStop,clearEmergency,settle:()=>motionRequests};')(
                document, window, (callback, ms) => timers.set(ms, callback), fetch);
        return {app, requests, tick: () => timers.get(100)(),
            down: () => elements.get('joystickBase').events.pointerdown({clientX:120,clientY:40}),
            up: () => windowEvents.pointerup(),
            block: () => { blockMove = true; }, release: () => releaseMove(),
            fail: () => { failStop = true; }};
    }
    const flush = async () => { for (let i = 0; i < 12; ++i) await Promise.resolve(); };
    let f = fixture();
    for (let i = 0; i < 5; ++i) f.tick();
    await flush(); assert(f.requests.length === 0, 'Idle page must not issue direct control');
    f.down(); f.tick(); await f.app.settle();
    assert(f.requests.some(p => p.startsWith('/move')), 'Active joystick sends motion');
    f.up(); await f.app.settle();
    assert(f.requests.some(p => p === '/move?move=0&rotation=0'), 'Release submits zero intent');

    f = fixture(); f.block();
    const moving = f.app.sendMove(20, 0); await flush();
    const targeting = f.app.sendTarget(); await flush();
    assert(!f.requests.some(p => p.startsWith('/target')), 'Target waits for prior move');
    f.release(); await moving; await targeting;
    const moves = f.requests.filter(p => p.startsWith('/move')).length;
    for (let i = 0; i < 5; ++i) f.tick(); f.up(); await flush();
    assert(f.requests.filter(p => p.startsWith('/move')).length === moves,
        'Idle interval and target-button pointerup must not overwrite target mode');
    assert(f.requests.some(p => p.startsWith('/target')), 'Target was submitted');

    f = fixture();
    const queued = f.app.sendMove(50, 0);
    await f.app.emergencyStop(); await queued;
    assert(!f.requests.some(p => p.startsWith('/move')), 'Stop cancels queued old motion');
    f.down(); f.tick(); await flush();
    assert(!f.requests.some(p => p.startsWith('/move')), 'Latched stop blocks joystick');
    await f.app.clearEmergency();
    for (let i = 0; i < 5; ++i) f.tick(); await flush();
    assert(!f.requests.some(p => p.startsWith('/move')), 'Clear requires a new operator gesture');
    f.down(); f.tick(); await f.app.settle();
    assert(f.requests.some(p => p.startsWith('/move')), 'New post-clear gesture permits motion');

    f = fixture(); f.block();
    const old = f.app.sendMove(20, 0); await flush();
    await f.app.emergencyStop();
    const clearing = f.app.clearEmergency(); await flush();
    assert(!f.requests.includes('/clear_estop'), 'Clear drains old in-flight motion while stopped');
    f.release(); await old; await clearing;
    assert(f.requests.includes('/clear_estop'), 'Clear occurs after old requests finish');

    f = fixture(); f.fail();
    await f.app.emergencyStop(); await f.app.clearEmergency();
    assert(!f.requests.includes('/clear_estop'), 'Failed stop must not lead to blind clear');
    return '5 browser control scenarios passed';
}
if (typeof require !== 'undefined' && require.main === module) {
    const fs = require('fs'), path = require('path');
    const html = fs.readFileSync(path.join(__dirname, '../../../T1-ESP-C6/src/controlApp.html'), 'utf8');
    runBrowserTests(html).then(message => console.log(message)).catch(error => { console.error(error); process.exitCode = 1; });
}
