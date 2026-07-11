// Telemetry player shared by Mission Control and the Final Report timeline.
//
// Two modes:
//   replay — a fixed frame array played against a wall clock, scrubbable, with
//            variable speed. Drives the report timeline and recorded missions.
//   live   — frames are appended as they arrive from the simulation WebSocket;
//            the player stays pinned to the newest frame.

export class Player {
  constructor(frames = [], mode = "replay") {
    this.frames = frames;
    this.mode = mode;
    this.index = 0;
    this.simTime = frames.length ? frames[0].t : 0;
    this.speed = 1;
    this.playing = false;
    this._subs = [];
    this._endSubs = [];
    this._raf = null;
    this._last = 0;
  }

  get duration() { return this.frames.length ? this.frames[this.frames.length - 1].t : 0; }
  get current() { return this.frames[this.index] || null; }

  onFrame(fn) { this._subs.push(fn); return this; }
  onEnd(fn) { this._endSubs.push(fn); return this; }
  _emit() { const f = this.current; if (f) this._subs.forEach((s) => s(f, this.index)); }

  // --- live -----------------------------------------------------------------
  append(frame) {
    this.frames.push(frame);
    this.index = this.frames.length - 1;
    this.simTime = frame.t;
    this._emit();
  }

  // --- replay ---------------------------------------------------------------
  play() {
    if (this.mode !== "replay" || this.playing) return;
    if (this.index >= this.frames.length - 1) this.seekTime(0);
    this.playing = true; this._last = performance.now();
    const tick = (now) => {
      if (!this.playing) return;
      const dt = (now - this._last) / 1000; this._last = now;
      this.simTime += dt * this.speed;
      let i = this.index;
      while (i < this.frames.length - 1 && this.frames[i + 1].t <= this.simTime) i++;
      if (i !== this.index) { this.index = i; this._emit(); }
      if (this.index >= this.frames.length - 1) { this.pause(); this._endSubs.forEach((s) => s()); return; }
      this._raf = requestAnimationFrame(tick);
    };
    this._raf = requestAnimationFrame(tick);
  }
  pause() { this.playing = false; cancelAnimationFrame(this._raf); }
  toggle() { this.playing ? this.pause() : this.play(); }

  setSpeed(s) { this.speed = s; }

  seekIndex(i) {
    this.index = Math.max(0, Math.min(this.frames.length - 1, i));
    this.simTime = this.frames[this.index]?.t ?? 0;
    this._emit();
  }
  seekTime(t) {
    let i = 0;
    while (i < this.frames.length - 1 && this.frames[i + 1].t <= t) i++;
    this.simTime = t;
    this.index = i;
    this._emit();
  }
  seekFraction(f) { this.seekTime(f * this.duration); }

  destroy() { this.pause(); this._subs = []; this._endSubs = []; }
}
