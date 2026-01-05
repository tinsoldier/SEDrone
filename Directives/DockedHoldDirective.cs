using System.Collections.Generic;

namespace IngameScript
{
    /// <summary>
    /// Holds position while docked and avoids auto-undocking on startup.
    /// </summary>
    public class DockedHoldDirective : IDirective
    {
        public string Name => "DockedHold";

        public IEnumerable<BehaviorIntent> Execute(DroneContext ctx)
        {
            if (ctx.IsDocked)
            {
                ctx.SetDampeners(false);
            }

            while (ctx.IsDocked)
            {
                yield return new BehaviorIntent
                {
                    Position = null,
                    Orientation = null,
                    ExitWhen = () => !ctx.IsDocked
                };
            }

            ctx.SetDampeners(true);
        }
    }
}
