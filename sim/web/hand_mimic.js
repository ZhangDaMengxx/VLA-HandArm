    this.statusElement.textContent = msg;
    if (this.statusCallback) {
      this.statusCallback(msg);
    }
  }
}
