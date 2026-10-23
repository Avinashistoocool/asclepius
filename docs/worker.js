// Web Worker wrapper: runs the compare() pipeline off the main thread and streams progress.
import { compare } from "./planner.js";

self.onmessage = (event) => {
  const { points, ...opts } = event.data || {};
  try {
    const data = compare(points, opts, (fraction, message) => self.postMessage({ type: "progress", fraction, message }));
    self.postMessage({ type: "result", data });
  } catch (error) {
    self.postMessage({ type: "error", error: error && error.message ? error.message : String(error) });
  }
};
