using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;

namespace IngameScript
{
    public interface ICommandBus
    {
        void Publish(DroneCommandMessage message);
        bool TryDequeue(out DroneCommandMessage message);
    }

    public interface IStateBus<T>
    {
        void Publish(T state);
        bool TryGetLatest(out T state);
    }

    public class IgcCommandBus : ICommandBus
    {
        private readonly IMyIntergridCommunicationSystem _igc;
        private readonly string _channel;
        private readonly IMyBroadcastListener _listener;

        public IgcCommandBus(IMyIntergridCommunicationSystem igc, string channel)
        {
            _igc = igc;
            _channel = channel;
            _listener = _igc?.RegisterBroadcastListener(_channel);
        }

        public void Publish(DroneCommandMessage message)
        {
            if (_igc == null || string.IsNullOrEmpty(_channel))
                return;

            _igc.SendBroadcastMessage(_channel, message.Serialize());
        }

        public bool TryDequeue(out DroneCommandMessage message)
        {
            message = new DroneCommandMessage();
            if (_listener == null)
                return false;

            while (_listener.HasPendingMessage)
            {
                var msg = _listener.AcceptMessage();
                var data = msg.Data as string;
                if (data == null)
                    continue;

                DroneCommandMessage parsed;
                if (DroneCommandMessage.TryParse(data, out parsed))
                {
                    message = parsed;
                    return true;
                }
            }

            return false;
        }
    }

    public class IgcStateBus : IStateBus<LeaderStateMessage>
    {
        private readonly IMyIntergridCommunicationSystem _igc;
        private readonly string _channel;
        private readonly IMyBroadcastListener _listener;
        private LeaderStateMessage _latest;
        private bool _hasLatest;
        private bool _hasNew;

        public IgcStateBus(IMyIntergridCommunicationSystem igc, string channel)
        {
            _igc = igc;
            _channel = channel;
            _listener = _igc?.RegisterBroadcastListener(_channel);
        }

        public void Publish(LeaderStateMessage state)
        {
            if (_igc == null || string.IsNullOrEmpty(_channel))
                return;

            _igc.SendBroadcastMessage(_channel, state.Serialize());
        }

        public bool TryGetLatest(out LeaderStateMessage state)
        {
            if (_listener != null)
            {
                while (_listener.HasPendingMessage)
                {
                    var msg = _listener.AcceptMessage();
                    var data = msg.Data as string;
                    if (data == null)
                        continue;

                    LeaderStateMessage parsed;
                    if (LeaderStateMessage.TryParse(data, out parsed))
                    {
                        _latest = parsed;
                        _hasLatest = true;
                        _hasNew = true;
                    }
                }
            }

            if (_hasNew)
            {
                _hasNew = false;
                state = _latest;
                return true;
            }

            state = _latest;
            return false;
        }
    }

    public class LocalCommandBus : ICommandBus
    {
        private readonly Queue<DroneCommandMessage> _queue = new Queue<DroneCommandMessage>();

        public void Publish(DroneCommandMessage message)
        {
            _queue.Enqueue(message);
        }

        public bool TryDequeue(out DroneCommandMessage message)
        {
            if (_queue.Count > 0)
            {
                message = _queue.Dequeue();
                return true;
            }

            message = new DroneCommandMessage();
            return false;
        }
    }

    public class LocalStateBus : IStateBus<LeaderStateMessage>
    {
        private LeaderStateMessage _latest;
        private bool _hasLatest;

        public void Publish(LeaderStateMessage state)
        {
            _latest = state;
            _hasLatest = true;
        }

        public bool TryGetLatest(out LeaderStateMessage state)
        {
            state = _latest;
            return _hasLatest;
        }
    }
}
