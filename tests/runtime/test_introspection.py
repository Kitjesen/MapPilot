"""Tests for runtime resource monitoring."""

# ============================================================================
# ResourceMonitor
# ============================================================================

class TestResourceMonitor:
    def test_instantiate(self):
        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor(poll_interval=1.0)
        assert m is not None

    def test_register_and_stats_empty(self):
        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor(poll_interval=60.0)
        m.register("test-proc", pid=1)  # PID 1 = init/system process
        # Stats not populated until first poll
        stats = m.stats()
        assert isinstance(stats, dict)

    def test_summary_no_procs(self):
        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor()
        s = m.summary()
        assert isinstance(s, str)
        assert "no processes" in s

    def test_start_stop(self):
        import os

        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor(poll_interval=60.0)
        m.register("self", pid=os.getpid())
        m.start()
        assert m.is_running()
        m.stop()

    def test_unregister(self):
        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor()
        m.register("proc-a", pid=1)
        m.unregister("proc-a")
        # Should not appear in future polls
        assert "proc-a" not in m._pids

    def test_start_idempotent(self):
        import os

        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor(poll_interval=60.0)
        m.register("self", pid=os.getpid())
        m.start()
        m.start()  # second call is no-op
        assert m.is_running()
        m.stop()

    def test_import_from_core(self):
        from runtime import ResourceMonitor  # lazy export via __getattr__
        assert ResourceMonitor is not None
