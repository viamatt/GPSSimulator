using GPSSimulator.Services;

namespace GPSSimulator
{
	public partial class App : Application
	{
		private readonly GpsSimulatorService _simService;

		public App(GpsSimulatorService simService)
		{
			_simService = simService;
			InitializeComponent();

			// Backstop for terminations that bypass window teardown.
			AppDomain.CurrentDomain.ProcessExit += (_, _) => StopTransmission();
		}

		protected override Window CreateWindow(IActivationState? activationState)
		{
			var window = new Window(new MainPage()) { Title = "GPSSimulator" };

			// If the user closes the window while a replay is still transmitting,
			// the HackRF is left streaming. libhackrf's hackrf_exit() then blocks
			// until the USB device is physically disconnected, so the process
			// appears to hang. Stop and dispose the device before teardown.
			window.Destroying += (_, _) => StopTransmission();

			return window;
		}

		/// <summary>
		/// Cancels any active transmission and waits (with a bounded timeout) for
		/// the HackRF stream and device to be disposed. Runs on a background thread
		/// and blocks the caller, because window teardown cannot await.
		/// </summary>
		private void StopTransmission()
		{
			if (!_simService.IsRunning) return;

			try
			{
				Task.Run(() => _simService.StopAndWaitAsync(TimeSpan.FromSeconds(5)))
					.Wait(TimeSpan.FromSeconds(6));
			}
			catch (Exception ex)
			{
				System.Diagnostics.Debug.WriteLine($"Shutdown stop failed: {ex.Message}");
			}
		}
	}
}
