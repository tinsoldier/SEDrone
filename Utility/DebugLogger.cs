using System;
using System.Collections.Generic;
using System.Text;

namespace IngameScript
{
    /// <summary>
    /// Simple debug logger that writes to the programmable block's Echo output.
    /// Displays newest messages first (reverse chronological order).
    /// </summary>
    public class DebugLogger
    {
        private readonly Action<string> _echoFunc;
        private readonly Func<double> _getGameTime;
        private readonly List<string> _messages;
        private int _lineCount;
        private const int MAX_LINES = 25;

        // Track repeated messages for duration display
        private string _lastMessage;
        private double _lastMessageStartTime;

        // Perf stats (updated externally, rendered in Flush)
        private double _perfWindowStart;
        private int _perfSampleCount;
        private double _perfSumMs;
        private double _perfMinMs = double.MaxValue;
        private double _perfMaxMs;
        private string _perfLine;
        private string _perfStatus;

        /// <summary>
        /// Creates a debug logger that writes to the PB's Echo output.
        /// </summary>
        /// <param name="echoFunc">The Echo function from the PB (usually context.Echo)</param>
        /// <param name="getGameTime">Function to get current game time in seconds</param>
        public DebugLogger(Action<string> echoFunc, Func<double> getGameTime = null)
        {
            _echoFunc = echoFunc;
            _getGameTime = getGameTime;
            _messages = new List<string>(MAX_LINES);
            _lineCount = 0;
            _lastMessage = null;
            _lastMessageStartTime = 0;
        }

        /// <summary>
        /// Logs a message to the debug buffer.
        /// If the same message is logged repeatedly, it will show duration instead of spamming.
        /// Messages are accumulated and written on Flush().
        /// </summary>
        public void Log(string message)
        {
            // Check if this is a repeated message
            if (_getGameTime != null && message == _lastMessage)
            {
                // Update the last message with duration
                double duration = _getGameTime() - _lastMessageStartTime;
                string durationMsg = $"[{_lineCount - 1:D3}] {message} ({duration:F0}s)";

                // Replace the last message with updated duration
                if (_messages.Count > 0)
                {
                    _messages[_messages.Count - 1] = durationMsg;
                }
            }
            else
            {
                // New message - add it
                _messages.Add($"[{_lineCount++:D3}] {message}");

                // Track for duration
                _lastMessage = message;
                if (_getGameTime != null)
                {
                    _lastMessageStartTime = _getGameTime();
                }

                // Keep only last MAX_LINES
                if (_messages.Count > MAX_LINES)
                {
                    _messages.RemoveAt(0); // Remove oldest
                }
            }
        }

        /// <summary>
        /// Writes all buffered debug messages to the PB's Echo output.
        /// Newest messages appear first.
        /// Call this once per frame/tick to display accumulated logs.
        /// </summary>
        public void Flush()
        {
            if (_messages.Count > 0 || !string.IsNullOrEmpty(_perfLine))
            {
                var output = new StringBuilder();
                output.AppendLine("=== Debug Log (newest first) ===");
                if (!string.IsNullOrEmpty(_perfLine))
                {
                    if (!string.IsNullOrEmpty(_perfStatus))
                        output.AppendLine($"{_perfLine} [{_perfStatus}]");
                    else
                        output.AppendLine(_perfLine);
                }

                // Print in reverse order (newest first)
                for (int i = _messages.Count - 1; i >= 0; i--)
                {
                    output.AppendLine(_messages[i]);
                }

                _echoFunc?.Invoke(output.ToString());
            }
        }

        /// <summary>
        /// Clears the debug buffer.
        /// </summary>
        public void Clear()
        {
            _messages.Clear();
            _lineCount = 0;
            _lastMessage = null;
            _lastMessageStartTime = 0;
            _perfLine = null;
            _perfStatus = null;
        }

        /// <summary>
        /// Updates perf stats for display (averaged over ~1 second).
        /// </summary>
        public void UpdatePerfStats(double sampleMs, double currentTime)
        {
            _perfSumMs += sampleMs;
            _perfSampleCount++;
            _perfMinMs = Math.Min(_perfMinMs, sampleMs);
            _perfMaxMs = Math.Max(_perfMaxMs, sampleMs);

            if (currentTime - _perfWindowStart < 1.0)
                return;

            double avg = _perfSampleCount > 0 ? _perfSumMs / _perfSampleCount : 0;
            _perfLine = $"avg {avg:F3}ms min {_perfMinMs:F3} max {_perfMaxMs:F3}";

            _perfWindowStart = currentTime;
            _perfSampleCount = 0;
            _perfSumMs = 0;
            _perfMinMs = double.MaxValue;
            _perfMaxMs = 0;
        }

        /// <summary>
        /// Sets a short status tag appended to the perf stats line.
        /// </summary>
        public void SetPerfStatus(string status)
        {
            _perfStatus = status;
        }

        /// <summary>
        /// Returns true if an echo function is available.
        /// </summary>
        public bool IsEnabled => _echoFunc != null;
    }
}
