using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;

namespace IngameScript
{
    /// <summary>
    /// Manages IGC request-response pattern for directives.
    /// Provides an await-like mechanism where directives can send requests
    /// and yield until a response is received.
    /// </summary>
    public class IGCRequestManager
    {
        private readonly IMyIntergridCommunicationSystem _igc;
        private readonly IMyProgrammableBlock _me;
        private readonly string _requestChannel;
        private readonly string _responseChannel;
        private readonly Action<string> _echo;
        private readonly long _droneEntityId;
        private readonly string _droneGridName;
        private readonly DockingPadManager _localDockingManager;

        private long _nextRequestId = 1;
        private Dictionary<long, PendingRequest> _pendingRequests = new Dictionary<long, PendingRequest>();
        private IMyBroadcastListener _responseListener;

        private class PendingRequest
        {
            public long RequestId;
            public double SentTime;
            public double Timeout;
            public object Response;
            public bool IsCompleted;
        }

        public IGCRequestManager(
            IMyIntergridCommunicationSystem igc,
            IMyProgrammableBlock me,
            string baseChannel,
            Action<string> echo = null,
            long droneEntityId = 0,
            string droneGridName = null,
            DockingPadManager localDockingManager = null)
        {
            _igc = igc;
            _me = me;
            _requestChannel = baseChannel + "_DOCK_REQUEST";
            _responseChannel = baseChannel + "_DOCK_RESPONSE";
            _echo = echo;
            _droneEntityId = droneEntityId != 0 ? droneEntityId : (_me != null ? _me.CubeGrid.EntityId : 0);
            _droneGridName = !string.IsNullOrEmpty(droneGridName)
                ? droneGridName
                : (_me != null ? _me.CubeGrid.CustomName : "");
            _localDockingManager = localDockingManager;

            // Register listener for responses
            if (_igc != null && _localDockingManager == null)
            {
                _responseListener = _igc.RegisterBroadcastListener(_responseChannel);
                _echo?.Invoke($"[IGC] Request manager initialized. Request:{_requestChannel} Response:{_responseChannel}");
            }
            else if (_localDockingManager != null)
            {
                _echo?.Invoke("[IGC] Request manager initialized (local docking)");
            }
        }

        /// <summary>
        /// Sends a docking pad request and returns a pending request handle.
        /// The directive should yield and check IsPending() until complete.
        /// </summary>
        /// <param name="currentTime">Current game time</param>
        /// <param name="dronePosition">Drone's current world position (for closest pad selection)</param>
        /// <param name="timeout">Request timeout in seconds</param>
        public PendingDockingRequest RequestDockingPad(double currentTime, VRageMath.Vector3D dronePosition, double timeout = 5.0)
        {
            long requestId = _nextRequestId++;

            var request = new DockingPadRequest
            {
                DroneEntityId = _droneEntityId,
                DroneGridName = _droneGridName,
                RequestId = requestId,
                DronePosition = dronePosition,
                Timestamp = currentTime
            };

            if (_localDockingManager != null)
            {
                var response = _localDockingManager.ProcessRequest(request, currentTime);
                var localPending = new PendingRequest
                {
                    RequestId = requestId,
                    SentTime = currentTime,
                    Timeout = timeout,
                    IsCompleted = true,
                    Response = response
                };
                _pendingRequests[requestId] = localPending;
                return new PendingDockingRequest(this, requestId);
            }

            if (_igc != null)
            {
                _igc.SendBroadcastMessage(_requestChannel, request.Serialize());
                _echo?.Invoke($"[IGC] Sent docking pad request ID:{requestId}");
            }

            // Track pending request
            var pending = new PendingRequest
            {
                RequestId = requestId,
                SentTime = currentTime,
                Timeout = timeout,
                IsCompleted = false
            };
            _pendingRequests[requestId] = pending;

            return new PendingDockingRequest(this, requestId);
        }

        /// <summary>
        /// Processes incoming IGC messages. Call this each tick.
        /// </summary>
        public void ProcessMessages(double currentTime)
        {
            if (_responseListener == null)
                return;

            // Process response messages
            while (_responseListener.HasPendingMessage)
            {
                var msg = _responseListener.AcceptMessage();
                var data = msg.Data as string;
                if (data != null)
                {
                    DockingPadResponse response;
                    if (DockingPadResponse.TryParse(data, out response))
                    {
                        // Check if this response is for our drone
                        if (response.DroneEntityId == _droneEntityId)
                        {
                            PendingRequest pending;
                            if (_pendingRequests.TryGetValue(response.RequestId, out pending))
                            {
                                pending.Response = response;
                                pending.IsCompleted = true;
                                _echo?.Invoke($"[IGC] Received docking pad response ID:{response.RequestId}");
                            }
                        }
                    }
                }
            }

            // Clean up timed-out requests
            var toRemove = new List<long>();
            foreach (var kvp in _pendingRequests)
            {
                if (currentTime - kvp.Value.SentTime > kvp.Value.Timeout)
                {
                    toRemove.Add(kvp.Key);
                    _echo?.Invoke($"[IGC] Request ID:{kvp.Key} timed out");
                }
            }
            foreach (var id in toRemove)
            {
                _pendingRequests.Remove(id);
            }
        }

        /// <summary>
        /// Gets the result of a pending request.
        /// </summary>
        internal bool TryGetResult(long requestId, out DockingPadResponse response)
        {
            response = default(DockingPadResponse);

            PendingRequest pending;
            if (!_pendingRequests.TryGetValue(requestId, out pending))
                return false;

            if (!pending.IsCompleted)
                return false;

            response = (DockingPadResponse)pending.Response;
            _pendingRequests.Remove(requestId); // Consume the response
            return true;
        }

        /// <summary>
        /// Checks if a request is still pending.
        /// </summary>
        internal bool IsPending(long requestId)
        {
            return _pendingRequests.ContainsKey(requestId) && !_pendingRequests[requestId].IsCompleted;
        }
    }

    /// <summary>
    /// Handle for a pending docking pad request.
    /// Directives can yield while IsPending is true, then call GetResult().
    /// </summary>
    public class PendingDockingRequest
    {
        private readonly IGCRequestManager _manager;
        private readonly long _requestId;

        internal PendingDockingRequest(IGCRequestManager manager, long requestId)
        {
            _manager = manager;
            _requestId = requestId;
        }

        /// <summary>
        /// Returns true while waiting for response.
        /// </summary>
        public bool IsPending => _manager.IsPending(_requestId);

        /// <summary>
        /// Attempts to get the response. Returns true if available.
        /// </summary>
        public bool TryGetResult(out DockingPadResponse response)
        {
            return _manager.TryGetResult(_requestId, out response);
        }
    }
}
