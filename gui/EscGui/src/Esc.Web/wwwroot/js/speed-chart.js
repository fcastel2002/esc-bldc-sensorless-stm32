const attachments = new WeakMap();

export function attach(element, dotNetReference) {
  const state = {
    dragging: false,
    requestedCursor: null,
    pendingEvent: null,
    animationFrame: 0,
  };

  const sendPosition = () => {
    state.animationFrame = 0;
    if (!state.pendingEvent) return;

    const rect = element.getBoundingClientRect();
    const ratio =
      rect.width > 0
        ? Math.min(
            1,
            Math.max(0, (state.pendingEvent.clientX - rect.left) / rect.width),
          )
        : 0;
    const cursor = state.pendingEvent.cursor;
    state.pendingEvent = null;
    dotNetReference.invokeMethodAsync("SetCursorFromRatio", ratio, cursor);
  };

  const queuePosition = (event) => {
    state.pendingEvent = {
      clientX: event.clientX,
      cursor: state.requestedCursor,
    };
    if (!state.animationFrame) {
      state.animationFrame = requestAnimationFrame(sendPosition);
    }
  };

  const pointerDown = (event) => {
    if (event.button !== 0) return;
    state.dragging = true;
    state.requestedCursor =
      event.target.closest("[data-cursor]")?.dataset.cursor ?? null;
    element.setPointerCapture(event.pointerId);
    queuePosition(event);
    event.preventDefault();
  };

  const pointerMove = (event) => {
    if (state.dragging) queuePosition(event);
  };

  const pointerUp = (event) => {
    if (!state.dragging) return;
    queuePosition(event);
    state.dragging = false;
    state.requestedCursor = null;
    if (element.hasPointerCapture(event.pointerId)) {
      element.releasePointerCapture(event.pointerId);
    }
  };

  element.addEventListener("pointerdown", pointerDown);
  element.addEventListener("pointermove", pointerMove);
  element.addEventListener("pointerup", pointerUp);
  element.addEventListener("pointercancel", pointerUp);
  attachments.set(element, { state, pointerDown, pointerMove, pointerUp });
}

export function detach(element) {
  const attachment = attachments.get(element);
  if (!attachment) return;

  element.removeEventListener("pointerdown", attachment.pointerDown);
  element.removeEventListener("pointermove", attachment.pointerMove);
  element.removeEventListener("pointerup", attachment.pointerUp);
  element.removeEventListener("pointercancel", attachment.pointerUp);
  if (attachment.state.animationFrame) {
    cancelAnimationFrame(attachment.state.animationFrame);
  }
  attachments.delete(element);
}

export async function downloadFileFromStream(fileName, streamReference) {
  const arrayBuffer = await streamReference.arrayBuffer();
  const blob = new Blob([arrayBuffer], { type: "text/csv;charset=utf-8" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = fileName;
  anchor.click();
  anchor.remove();
  URL.revokeObjectURL(url);
}
