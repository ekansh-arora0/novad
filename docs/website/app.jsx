/* ═══════════════════════════════════════════════════════════════════════════
   NOVAD REACT APP
   Complete documentation website with React (via CDN)
   ═══════════════════════════════════════════════════════════════════════════ */

const { useState, useEffect } = React;

// ═══════════════════════════════════════════════════════════════════════════
// CODE EXAMPLES
// ═══════════════════════════════════════════════════════════════════════════

const codeExamples = {
    quickStart: `// 1. Configure Novad (do this once)
NovadConfig config = new NovadConfig.Builder()
    .leftFrontMotorName("left_front_drive")
    .leftRearMotorName("left_back_drive")
    .rightFrontMotorName("right_front_drive")
    .rightRearMotorName("right_back_drive")
    .leftFrontMotorDirection(MotorDirection.REVERSE)
    .leftRearMotorDirection(MotorDirection.REVERSE)
    .rightFrontMotorDirection(MotorDirection.FORWARD)
    .rightRearMotorDirection(MotorDirection.FORWARD)
    .translationalPIDFCoefficients(new PIDFCoefficients(0.046, 0, 0.01, 0.02))
    .headingPIDFCoefficients(new PIDFCoefficients(0.67, 0, 0.006, 0.02))
    .build();

// 2. Create Novad
Novad novad = new Novad(odometry, drivetrain, config);

// 3. Use in your TeleOp loop - that's it!
while (opModeIsActive()) {
    novad.defense(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
}`,

    configExample: `NovadConfig config = new NovadConfig.Builder()
    // Motor names (from your robot config)
    .leftFrontMotorName("left_front_drive")
    .leftRearMotorName("left_back_drive")
    .rightFrontMotorName("right_front_drive")
    .rightRearMotorName("right_back_drive")
    
    // Motor directions (left side typically reversed)
    .leftFrontMotorDirection(MotorDirection.REVERSE)
    .leftRearMotorDirection(MotorDirection.REVERSE)
    .rightFrontMotorDirection(MotorDirection.FORWARD)
    .rightRearMotorDirection(MotorDirection.FORWARD)
    
    // PIDF coefficients (tune these with the tuner OpModes!)
    .translationalPIDFCoefficients(new PIDFCoefficients(0.046, 0, 0.01, 0.02))
    .headingPIDFCoefficients(new PIDFCoefficients(0.67, 0, 0.006, 0.02))
    .velocityPIDFCoefficients(new PIDFCoefficients(0.02, 0, 0.001, 0.01))
    
    // Thresholds
    .movementThreshold(0.5)       // inches before defense kicks in
    .headingThreshold(0.035)      // radians (~2 degrees)
    .driverOverrideThreshold(0.1) // joystick deadzone
    .maxCorrectionPower(0.8)      // max motor power for corrections
    
    .build();`,

    tunerExample: `// In FTC Dashboard, find these variables and adjust them:
public static double HEADING_P = 0.67;   // Start here
public static double HEADING_I = 0.0;    // Usually keep at 0
public static double HEADING_D = 0.006;  // Add if oscillating

// Tuning process:
// 1. Set I and D to 0
// 2. Increase P until robot resists rotation
// 3. If it oscillates, reduce P or add D
// 4. Fine tune until smooth`
};

// ═══════════════════════════════════════════════════════════════════════════
// COMPONENTS
// ═══════════════════════════════════════════════════════════════════════════

function Navbar({ currentPage, setPage }) {
    return (
        <nav className="navbar">
            <div className="nav-container">
                <a href="#" className="nav-logo" onClick={() => setPage('home')}>
                    <span>🛡️</span>
                    <span>Novad</span>
                </a>
                <div className="nav-links">
                    <a href="#" className={currentPage === 'home' ? 'active' : ''} onClick={() => setPage('home')}>Home</a>
                    <a href="#" className={currentPage === 'quickstart' ? 'active' : ''} onClick={() => setPage('quickstart')}>Quickstart</a>
                    <a href="#" className={currentPage === 'tuning' ? 'active' : ''} onClick={() => setPage('tuning')}>Tuning</a>
                    <a href="#" className={currentPage === 'api' ? 'active' : ''} onClick={() => setPage('api')}>API</a>
                </div>
            </div>
        </nav>
    );
}

function CodeBlock({ title, code, language = 'java' }) {
    useEffect(() => {
        Prism.highlightAll();
    }, [code]);

    return (
        <div className="code-block">
            {title && (
                <div className="code-header">
                    <span className="code-title">{title}</span>
                </div>
            )}
            <pre><code className={`language-${language}`}>{code}</code></pre>
        </div>
    );
}

function Alert({ type, children }) {
    const icons = { info: 'ℹ️', warning: '⚠️', success: '✅' };
    return (
        <div className={`alert alert-${type}`}>
            <span>{icons[type]}</span>
            <div>{children}</div>
        </div>
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// HOME PAGE
// ═══════════════════════════════════════════════════════════════════════════

function HomePage({ setPage }) {
    return (
        <>
            <section className="hero">
                <div className="hero-content">
                    <div className="hero-badge">FTC Defense Library v1.0</div>
                    <h1 className="hero-title">
                        Stop Getting<br />
                        <span className="gradient-text">Pushed Around</span>
                    </h1>
                    <p className="hero-subtitle">
                        Add push resistance to your FTC robot with a single line of code. 
                        Novad uses odometry to detect unwanted movement and automatically applies counter-force.
                    </p>
                    <div className="hero-buttons">
                        <button className="btn btn-primary" onClick={() => setPage('quickstart')}>
                            Get Started →
                        </button>
                        <a href="https://github.com/novad/novad" className="btn btn-secondary" target="_blank">
                            View on GitHub
                        </a>
                    </div>
                </div>
            </section>

            <section id="features">
                <h2 className="section-title">Why Novad?</h2>
                <p className="section-subtitle">Built by FTC teams, for FTC teams.</p>
                
                <div className="features-grid">
                    <div className="feature-card">
                        <div className="feature-icon">⚡</div>
                        <h3>One Line Integration</h3>
                        <p>Just call novad.defense() in your TeleOp loop. That's it. We handle the rest.</p>
                    </div>
                    <div className="feature-card">
                        <div className="feature-icon">🎯</div>
                        <h3>Dual-Layer PID</h3>
                        <p>Position + velocity correction for instant response and accurate return to position.</p>
                    </div>
                    <div className="feature-card">
                        <div className="feature-icon">🔧</div>
                        <h3>Live Tuning</h3>
                        <p>Tune PID values in real-time with FTC Dashboard. No redeploying needed.</p>
                    </div>
                    <div className="feature-card">
                        <div className="feature-icon">🎮</div>
                        <h3>Driver Override</h3>
                        <p>Automatically detects joystick input and steps aside. You're always in control.</p>
                    </div>
                    <div className="feature-card">
                        <div className="feature-icon">🔒</div>
                        <h3>Position Lock</h3>
                        <p>Lock your position for maximum defense. Perfect for end-game scoring protection.</p>
                    </div>
                    <div className="feature-card">
                        <div className="feature-icon">📊</div>
                        <h3>Rich Telemetry</h3>
                        <p>See exactly what Novad is doing with detailed position, error, and correction data.</p>
                    </div>
                </div>
            </section>

            <section>
                <h2 className="section-title">Quick Example</h2>
                <CodeBlock title="NovadTeleOp.java" code={codeExamples.quickStart} />
            </section>
        </>
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// QUICKSTART PAGE
// ═══════════════════════════════════════════════════════════════════════════

function QuickstartPage() {
    return (
        <section style={{ paddingTop: '7rem' }}>
            <h1 className="section-title">Quickstart Guide</h1>
            <p className="section-subtitle">Get Novad running on your robot in 10 minutes.</p>

            <Alert type="info">
                <strong>New to Novad?</strong> This guide walks you through the complete setup process, 
                from installation to your first tuned defense system.
            </Alert>

            <div className="steps">
                <div className="step">
                    <div className="step-number">1</div>
                    <div className="step-content">
                        <h3>Clone the Repository</h3>
                        <p>Clone the FtcRobotController with Novad already integrated:</p>
                        <CodeBlock 
                            code="git clone https://github.com/novad/FtcRobotController-Novad.git" 
                            language="bash"
                        />
                        <p>Or manually copy the <code>novad</code> folder into your TeamCode.</p>
                    </div>
                </div>

                <div className="step">
                    <div className="step-number">2</div>
                    <div className="step-content">
                        <h3>Configure Your Robot</h3>
                        <p>Create a NovadConfig with your robot's motor names and directions:</p>
                        <CodeBlock title="RobotConfig.java" code={codeExamples.configExample} />
                    </div>
                </div>

                <div className="step">
                    <div className="step-number">3</div>
                    <div className="step-content">
                        <h3>Set Up Odometry</h3>
                        <p>Novad needs to know where your robot is. Use our ThreeWheelOdometry adapter or connect your existing system:</p>
                        <CodeBlock code={`// Three-wheel odometry setup
ThreeWheelOdometry odometry = new ThreeWheelOdometry(
    () -> leftEncoder.getCurrentPosition(),
    () -> rightEncoder.getCurrentPosition(),
    () -> centerEncoder.getCurrentPosition(),
    1.89,    // wheel diameter (inches)
    8192,    // ticks per revolution
    14.0,    // track width (inches)
    6.0      // forward offset (inches)
);`} />
                    </div>
                </div>

                <div className="step">
                    <div className="step-number">4</div>
                    <div className="step-content">
                        <h3>Run the Tuners</h3>
                        <p>Deploy and run the tuning OpModes to find the right PID values for your robot:</p>
                        <ul style={{ marginTop: '0.5rem', marginLeft: '1.5rem', color: 'var(--text-muted)' }}>
                            <li><strong>NovadHeadingTuner</strong> — Tune rotation resistance</li>
                            <li><strong>NovadTranslationalTuner</strong> — Tune X/Y position hold</li>
                            <li><strong>NovadFullTuner</strong> — Fine-tune everything together</li>
                        </ul>
                    </div>
                </div>

                <div className="step">
                    <div className="step-number">5</div>
                    <div className="step-content">
                        <h3>Use in TeleOp</h3>
                        <p>Add Novad to your TeleOp and you're done!</p>
                        <CodeBlock code={`// In your TeleOp loop
while (opModeIsActive()) {
    // This one line handles everything!
    novad.defense(
        gamepad1.left_stick_x,
        -gamepad1.left_stick_y,
        gamepad1.right_stick_x
    );
}`} />
                    </div>
                </div>
            </div>

            <Alert type="success">
                <strong>You're ready!</strong> Your robot will now resist being pushed. 
                Continue to the Tuning page to optimize your defense.
            </Alert>
        </section>
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// TUNING PAGE
// ═══════════════════════════════════════════════════════════════════════════

function TuningPage() {
    return (
        <section style={{ paddingTop: '7rem' }}>
            <h1 className="section-title">Tuning Guide</h1>
            <p className="section-subtitle">Get the most out of Novad with proper PID tuning.</p>

            <Alert type="warning">
                <strong>Important:</strong> Novad works best when tuned for YOUR specific robot. 
                Take 15-20 minutes to tune properly — it makes a huge difference!
            </Alert>

            <h2 style={{ marginTop: '2rem' }}>Understanding PIDF</h2>
            
            <table className="tuning-table">
                <thead>
                    <tr>
                        <th>Term</th>
                        <th>What It Does</th>
                        <th>When to Increase</th>
                        <th>When to Decrease</th>
                    </tr>
                </thead>
                <tbody>
                    <tr>
                        <td><code>P</code></td>
                        <td>Proportional — reacts to current error</td>
                        <td>Robot doesn't resist enough</td>
                        <td>Robot oscillates/vibrates</td>
                    </tr>
                    <tr>
                        <td><code>I</code></td>
                        <td>Integral — fixes accumulated error</td>
                        <td>Robot doesn't return to exact position</td>
                        <td>Robot overcorrects or winds up</td>
                    </tr>
                    <tr>
                        <td><code>D</code></td>
                        <td>Derivative — dampens oscillation</td>
                        <td>Robot overshoots then oscillates</td>
                        <td>Response feels sluggish</td>
                    </tr>
                    <tr>
                        <td><code>F</code></td>
                        <td>Feedforward — constant boost</td>
                        <td>Robot needs minimum power to move</td>
                        <td>Robot moves when it shouldn't</td>
                    </tr>
                </tbody>
            </table>

            <h2 style={{ marginTop: '3rem' }}>Tuning Process</h2>

            <div className="card" style={{ marginTop: '1.5rem' }}>
                <div className="card-header">
                    <span style={{ fontSize: '1.5rem' }}>🎯</span>
                    <h3>Step 1: Tune Heading First</h3>
                </div>
                <ol style={{ marginLeft: '1.5rem', color: 'var(--text-muted)' }}>
                    <li>Deploy and run <strong>NovadHeadingTuner</strong></li>
                    <li>Open FTC Dashboard at <code>http://192.168.43.1:8080/dash</code></li>
                    <li>Find the <code>NovadHeadingTuner</code> config section</li>
                    <li>Set <code>HEADING_I</code> and <code>HEADING_D</code> to 0</li>
                    <li>Slowly increase <code>HEADING_P</code> until robot resists rotation</li>
                    <li>If oscillating, reduce P or add small D</li>
                    <li>Note your final values!</li>
                </ol>
                <Alert type="info">
                    <strong>Starting values:</strong> HEADING_P = 0.67, HEADING_I = 0, HEADING_D = 0.006
                </Alert>
            </div>

            <div className="card" style={{ marginTop: '1.5rem' }}>
                <div className="card-header">
                    <span style={{ fontSize: '1.5rem' }}>📍</span>
                    <h3>Step 2: Tune Translational</h3>
                </div>
                <ol style={{ marginLeft: '1.5rem', color: 'var(--text-muted)' }}>
                    <li>Deploy and run <strong>NovadTranslationalTuner</strong></li>
                    <li>Same process: Start with P only</li>
                    <li>Push the robot and see if it pushes back</li>
                    <li>Increase P until it resists being pushed</li>
                    <li>Add D if it oscillates</li>
                </ol>
                <Alert type="info">
                    <strong>Starting values:</strong> POS_P = 0.046, POS_I = 0, POS_D = 0.01
                </Alert>
            </div>

            <div className="card" style={{ marginTop: '1.5rem' }}>
                <div className="card-header">
                    <span style={{ fontSize: '1.5rem' }}>⚡</span>
                    <h3>Step 3: Fine Tune Everything</h3>
                </div>
                <ol style={{ marginLeft: '1.5rem', color: 'var(--text-muted)' }}>
                    <li>Run <strong>NovadFullTuner</strong></li>
                    <li>Enter your tuned values from steps 1-2</li>
                    <li>Test all directions: push forward, sideways, rotate</li>
                    <li>Make small adjustments until smooth</li>
                    <li>Copy final values to your TeleOp config!</li>
                </ol>
            </div>

            <h2 style={{ marginTop: '3rem' }}>Using FTC Dashboard</h2>
            
            <div className="features-grid">
                <div className="feature-card">
                    <div className="feature-icon">🌐</div>
                    <h3>Connect to Dashboard</h3>
                    <p>Connect to robot WiFi, then open <code>192.168.43.1:8080/dash</code> or use <a href="https://panels.bylazar.com" target="_blank">panels.bylazar.com</a></p>
                </div>
                <div className="feature-card">
                    <div className="feature-icon">📊</div>
                    <h3>Live Graphs</h3>
                    <p>Watch position error and correction values in real-time to see how your tuning affects response.</p>
                </div>
                <div className="feature-card">
                    <div className="feature-icon">🎚️</div>
                    <h3>Adjust on the Fly</h3>
                    <p>Change PID values without redeploying. Changes take effect immediately!</p>
                </div>
            </div>
        </section>
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// API PAGE
// ═══════════════════════════════════════════════════════════════════════════

function ApiPage() {
    return (
        <section style={{ paddingTop: '7rem' }}>
            <h1 className="section-title">API Reference</h1>
            <p className="section-subtitle">Complete documentation of all Novad classes and methods.</p>

            <h2 style={{ marginTop: '2rem' }}>Main Classes</h2>

            <div className="card" style={{ marginTop: '1.5rem' }}>
                <div className="card-header">
                    <h3>Novad</h3>
                </div>
                <p style={{ color: 'var(--text-muted)', marginBottom: '1rem' }}>Main entry point for the defense system.</p>
                
                <table className="tuning-table">
                    <thead>
                        <tr><th>Method</th><th>Description</th></tr>
                    </thead>
                    <tbody>
                        <tr>
                            <td><code>defense(x, y, rotation)</code></td>
                            <td>Main method — call every loop. Handles defense + driver input.</td>
                        </tr>
                        <tr>
                            <td><code>lockPosition()</code></td>
                            <td>Lock current position for maximum resistance.</td>
                        </tr>
                        <tr>
                            <td><code>unlockPosition()</code></td>
                            <td>Release position lock.</td>
                        </tr>
                        <tr>
                            <td><code>togglePositionLock()</code></td>
                            <td>Toggle lock on/off (good for button binding).</td>
                        </tr>
                        <tr>
                            <td><code>enable() / disable()</code></td>
                            <td>Turn defense on or off.</td>
                        </tr>
                        <tr>
                            <td><code>setPositionPID(p, i, d)</code></td>
                            <td>Set translational PID gains.</td>
                        </tr>
                        <tr>
                            <td><code>setHeadingPID(p, i, d)</code></td>
                            <td>Set heading PID gains.</td>
                        </tr>
                        <tr>
                            <td><code>setVelocityPID(p, i, d)</code></td>
                            <td>Set velocity PID gains.</td>
                        </tr>
                    </tbody>
                </table>
            </div>

            <div className="card" style={{ marginTop: '1.5rem' }}>
                <div className="card-header">
                    <h3>NovadConfig.Builder</h3>
                </div>
                <p style={{ color: 'var(--text-muted)', marginBottom: '1rem' }}>Configure your robot with the builder pattern.</p>
                
                <table className="tuning-table">
                    <thead>
                        <tr><th>Method</th><th>Description</th></tr>
                    </thead>
                    <tbody>
                        <tr>
                            <td><code>leftFrontMotorName(name)</code></td>
                            <td>Set motor name from robot config.</td>
                        </tr>
                        <tr>
                            <td><code>leftFrontMotorDirection(dir)</code></td>
                            <td>Set motor direction (FORWARD/REVERSE).</td>
                        </tr>
                        <tr>
                            <td><code>translationalPIDFCoefficients(pidf)</code></td>
                            <td>Set position hold PIDF values.</td>
                        </tr>
                        <tr>
                            <td><code>headingPIDFCoefficients(pidf)</code></td>
                            <td>Set rotation hold PIDF values.</td>
                        </tr>
                        <tr>
                            <td><code>movementThreshold(inches)</code></td>
                            <td>Minimum movement before defense activates.</td>
                        </tr>
                        <tr>
                            <td><code>maxCorrectionPower(power)</code></td>
                            <td>Maximum motor power for corrections (0-1).</td>
                        </tr>
                        <tr>
                            <td><code>build()</code></td>
                            <td>Create the NovadConfig instance.</td>
                        </tr>
                    </tbody>
                </table>
            </div>

            <div className="card" style={{ marginTop: '1.5rem' }}>
                <div className="card-header">
                    <h3>PIDFCoefficients</h3>
                </div>
                <CodeBlock code={`// Create PIDF coefficients
PIDFCoefficients translational = new PIDFCoefficients(
    0.046,  // P - proportional
    0.0,    // I - integral
    0.01,   // D - derivative
    0.02    // F - feedforward
);

// Or without feedforward
PIDFCoefficients heading = new PIDFCoefficients(0.67, 0, 0.006);`} />
            </div>

            <h2 style={{ marginTop: '3rem' }}>Interfaces</h2>

            <div className="card" style={{ marginTop: '1.5rem' }}>
                <div className="card-header">
                    <h3>NovadOdometry</h3>
                </div>
                <p style={{ color: 'var(--text-muted)', marginBottom: '1rem' }}>Implement this to connect your odometry system.</p>
                <CodeBlock code={`public interface NovadOdometry {
    Vector2D getPosition();      // Current (x, y) in inches
    double getHeading();         // Current heading in radians
    Vector2D getVelocity();      // Current velocity
    void update();               // Update readings
    void setPose(x, y, heading); // Set position
}`} />
            </div>

            <div className="card" style={{ marginTop: '1.5rem' }}>
                <div className="card-header">
                    <h3>NovadDrivetrain</h3>
                </div>
                <p style={{ color: 'var(--text-muted)', marginBottom: '1rem' }}>Implement this to connect your drivetrain.</p>
                <CodeBlock code={`public interface NovadDrivetrain {
    void drive(forward, strafe, rotation);  // Main drive method
    void setMotorPowers(fl, fr, bl, br);    // Direct motor control
    void stop();                             // Stop all motors
}`} />
            </div>
        </section>
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// APP
// ═══════════════════════════════════════════════════════════════════════════

function App() {
    const [currentPage, setPage] = useState('home');

    // Scroll to top when page changes
    const handlePageChange = (page) => {
        setPage(page);
        window.scrollTo(0, 0);
    };

    const renderPage = () => {
        switch (currentPage) {
            case 'quickstart': return <QuickstartPage />;
            case 'tuning': return <TuningPage />;
            case 'api': return <ApiPage />;
            default: return <HomePage setPage={handlePageChange} />;
        }
    };

    return (
        <>
            <Navbar currentPage={currentPage} setPage={handlePageChange} />
            <main style={{ minHeight: '100vh', paddingBottom: '4rem' }}>
                {renderPage()}
            </main>
            <footer className="footer">
                <p>🛡️ Novad Defense Library — Built for FTC Teams</p>
                <p style={{ marginTop: '0.5rem', fontSize: '0.875rem' }}>Open source under MIT License</p>
            </footer>
        </>
    );
}

// Render
ReactDOM.createRoot(document.getElementById('root')).render(<App />);
