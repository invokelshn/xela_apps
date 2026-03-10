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
}) {
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
    closeExisting();

    if (candidateIndex >= wsCandidates.length) {
      state.connected = false;
      state.activeWsUrl = null;
      setStatus("Disconnected (WS unreachable)", false);
      if (topicInfoEl) {
        topicInfoEl.textContent = topicInfoTextFor("");
      }
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

  return {
    connect,
    close: closeExisting,
  };
}
