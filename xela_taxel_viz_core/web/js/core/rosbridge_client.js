export function createRosbridgeClient({
  state,
  wsCandidates,
  topicInfoEl,
  topicInfoTextFor,
  setStatus,
  onOpen,
  onCloseOpened,
  onErrorOpened,
  onMessageParsed,
  // Every candidate can fail (server not up yet) or an established connection can drop (e.g. the
  // backend container restarting) with zero visible indication for callers that pass a no-op
  // setStatus (see xela_atag_taxel_viewer's dedicated connection in index.html) -- without this,
  // such a client is permanently dead for the rest of the page's life, silently, once its single
  // candidate fails once. Auto-retry from candidate 0 after a fixed delay closes that gap.
  retryDelayMs = 3000,
}) {
  let retryTimer = null;
  // Topics we've been asked to advertise, replayed on every (re)connect so a dropped/retried
  // WS doesn't silently leave rosbridge without our advertisement (see connect()/ws.onopen below).
  const desiredAdvertisements = new Map(); // topic -> type

  function clearRetryTimer() {
    if (retryTimer !== null) {
      clearTimeout(retryTimer);
      retryTimer = null;
    }
  }

  function scheduleRetry() {
    if (state.manualClose || retryTimer !== null) {
      return;
    }
    retryTimer = setTimeout(() => {
      retryTimer = null;
      connect(0);
    }, retryDelayMs);
  }

  function closeExisting() {
    if (!state.ws) {
      return;
    }
    try {
      state.ws.onclose = null;
      state.ws.onerror = null;
      state.ws.onmessage = null;
      state.ws.close();
    } catch (_) {
      // Ignore close errors.
    }
    state.ws = null;
  }

  function connect(candidateIndex = 0) {
    clearRetryTimer();
    state.manualClose = false;
    closeExisting();

    if (candidateIndex >= wsCandidates.length) {
      state.connected = false;
      state.activeWsUrl = null;
      setStatus("Disconnected (WS unreachable)", false);
      if (topicInfoEl) {
        topicInfoEl.textContent = topicInfoTextFor("");
      }
      scheduleRetry();
      return;
    }

    const wsUrl = wsCandidates[candidateIndex];
    setStatus(`Connecting ${candidateIndex + 1}/${wsCandidates.length}...`, false);
    if (topicInfoEl) {
      topicInfoEl.textContent = topicInfoTextFor(wsUrl);
    }

    const ws = new WebSocket(wsUrl);
    state.ws = ws;
    let opened = false;

    ws.onopen = () => {
      opened = true;
      state.connected = true;
      state.activeWsUrl = wsUrl;
      setStatus("Connected", true);
      for (const [topic, type] of desiredAdvertisements) {
        sendAdvertise(topic, type);
      }
      if (typeof onOpen === "function") {
        onOpen(wsUrl, ws);
      }
    };

    ws.onclose = () => {
      state.connected = false;
      if (!opened) {
        connect(candidateIndex + 1);
        return;
      }
      if (typeof onCloseOpened === "function") {
        onCloseOpened();
      } else {
        setStatus("Disconnected", false);
      }
      scheduleRetry();
    };

    ws.onerror = () => {
      state.connected = false;
      if (!opened) {
        return;
      }
      if (typeof onErrorOpened === "function") {
        onErrorOpened();
      } else {
        setStatus("Socket error", false);
      }
    };

    ws.onmessage = (ev) => {
      let msg;
      try {
        msg = JSON.parse(ev.data);
      } catch (_) {
        return;
      }
      if (!msg) {
        return;
      }
      if (typeof onMessageParsed === "function") {
        onMessageParsed(msg);
      }
    };
  }

  function sendServiceRequest(serviceName, args) {
    if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
      console.warn("Cannot send service request - WS not connected", serviceName);
      return;
    }
    const id = "call_" + Math.random().toString(36).substring(2, 9);
    const req = {
      op: "call_service",
      id: id,
      service: serviceName,
      args: args
    };
    state.ws.send(JSON.stringify(req));
  }

  function sendAdvertise(topic, type) {
    if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
      return;
    }
    state.ws.send(JSON.stringify({ op: "advertise", topic, type }));
  }

  // Registers `topic` for outbound publishing. Safe to call once at startup -- the advertisement
  // is remembered and replayed automatically on every reconnect (see ws.onopen above).
  function advertiseTopic(topic, type) {
    desiredAdvertisements.set(topic, type);
    sendAdvertise(topic, type);
  }

  function publishTopic(topic, msg) {
    if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
      console.warn("Cannot publish - WS not connected", topic);
      return;
    }
    state.ws.send(JSON.stringify({ op: "publish", topic, msg }));
  }

  return {
    connect,
    close: () => {
      state.manualClose = true;
      clearRetryTimer();
      closeExisting();
    },
    sendServiceRequest,
    advertiseTopic,
    publishTopic,
  };
}
