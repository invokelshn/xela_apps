export function createUiFeedback({ state, statusTextEl, statusDotEl, hintTextEl, modeInfoEl }) {
  function setStatus(text, ok) {
    statusTextEl.textContent = text;
    statusDotEl.classList.toggle("ok", !!ok);
  }

  function setHintText(text) {
    if (state.lastHintText !== text) {
      hintTextEl.textContent = text;
      state.lastHintText = text;
    }
  }

  function setModeInfoText(text) {
    if (state.lastModeInfoText !== text) {
      modeInfoEl.textContent = text;
      state.lastModeInfoText = text;
    }
  }

  return {
    setStatus,
    setHintText,
    setModeInfoText,
  };
}
