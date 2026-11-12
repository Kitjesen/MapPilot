# Evidence of a working model connection

- Configuration: a key exists or a user authorization is saved.
- Reachability: an actual request reaches the provider. A browser login page
  opening proves only the login entry is reachable.
- Inference: the selected model returns a completed response to a real request.
- Tool use: the model requests a declared tool, the real tool returns its result,
  and the model produces a response grounded in that result.

Report the tested model and measured duration of that request. Do not generalize
one request to every model under the provider or treat a demo fixture as live evidence.
HTTP authentication, quota, unsupported-model and timeout failures are different;
report the observed error without exposing credentials.
