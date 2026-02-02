const { useState, useEffect } = React;

// ═══════════════════════════════════════════════════════════════════════════
// CODE EXAMPLES
// ═══════════════════════════════════════════════════════════════════════════

const setupCode = `// NovadSetup.java - This is the ONLY file you edit!

// STEP 1: Your motor names (copy from Robot Configuration)
public static String FRONT_LEFT  = "frontLeft";
public static String FRONT_RIGHT = "frontRight";
public static String BACK_LEFT   = "backLeft";
public static String BACK_RIGHT  = "backRight";

// STEP 2: Motor directions (flip if a motor runs backwards)
public static boolean FRONT_LEFT_REVERSED  = true;
public static boolean FRONT_RIGHT_REVERSED = false;
public static boolean BACK_LEFT_REVERSED   = true;
public static boolean BACK_RIGHT_REVERSED  = false;

// STEP 3: Odometry type
public static boolean USE_PINPOINT = true;  // GoBilda Pinpoint
public static String PINPOINT_NAME = "pinpoint";`;

const teleOpCode = `// That's it! NovadTeleOp.java is ready to use.
// Just select "Novad TeleOp" on your Driver Station.

// Controls:
// • Left Stick  = Drive
// • Right Stick = Rotate  
// • A Button    = Toggle position lock
// • B Button    = Toggle defense on/off`;

const tuningCode = `// All values in NovadSetup.java are tunable in FTC Dashboard!
// Connect to: http://192.168.43.1:8080/dash
// Or use: panels.bylazar.com

// Find "NovadSetup" in the sidebar, then adjust:
public static double POSITION_P = 0.046;  // Push resistance
public static double HEADING_P  = 0.67;   // Rotation resistance

// Predictive Defense (makes response faster!)
public static boolean PREDICTIVE_ENABLED = true;
public static double PREDICTION_MS = 50;`;

// ═══════════════════════════════════════════════════════════════════════════
// COMPONENTS
// ═══════════════════════════════════════════════════════════════════════════

function CodeBlock({ code }) {
    useEffect(() => { Prism.highlightAll(); }, [code]);
    return (
        <pre className="code-block">
            <code className="language-java">{code}</code>
        </pre>
    );
}

function Navbar({ currentPage, setPage }) {
    return (
        <nav className="navbar">
            <div className="nav-container">
                <a href="#" className="nav-logo" onClick={(e) => { e.preventDefault(); setPage('home'); }}>
                    <span>🛡️</span>
                    <span>Novad</span>
                </a>
                <div className="nav-links">
                    <a href="#" className={currentPage === 'home' ? 'active' : ''} 
                       onClick={(e) => { e.preventDefault(); setPage('home'); }}>Home</a>
                    <a href="#" className={currentPage === 'setup' ? 'active' : ''} 
                       onClick={(e) => { e.preventDefault(); setPage('setup'); }}>Setup</a>
                    <a href="#" className={currentPage === 'tuning' ? 'active' : ''} 
                       onClick={(e) => { e.preventDefault(); setPage('tuning'); }}>Tuning</a>
                </div>
            </div>
        </nav>
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
                    <h1 className="hero-title">
                        Stop Getting<br />
                        <span className="gradient-text">Pushed Around</span>
                    </h1>
                    <p className="hero-subtitle">
                        Novad automatically keeps your FTC robot in place when opponents try to push you.
                        Setup takes 5 minutes.
                    </p>
                    <div className="hero-buttons">
                        <button className="btn btn-primary" onClick={() => setPage('setup')}>
                            Get Started →
                        </button>
                    </div>
                </div>
            </section>

            <section>
                <h2 className="section-title">How It Works</h2>
                <div className="features-grid">
                    <div className="feature-card">
                        <div className="feature-icon">📍</div>
                        <h3>Detects Movement</h3>
                        <p>Uses odometry to know when you're being pushed</p>
                    </div>
                    <div className="feature-card">
                        <div className="feature-icon">⚡</div>
                        <h3>Instant Response</h3>
                        <p>Predicts pushes and counters BEFORE you move</p>
                    </div>
                    <div className="feature-card">
                        <div className="feature-icon">🎮</div>
                        <h3>Driver Override</h3>
                        <p>Touch the joystick and Novad steps aside</p>
                    </div>
                </div>
            </section>

            <section>
                <h2 className="section-title">5-Minute Setup</h2>
                <div className="steps">
                    <div className="step">
                        <div className="step-number">1</div>
                        <div className="step-content">
                            <h3>Copy Files</h3>
                            <p>Copy <code>novad/</code>, <code>NovadSetup.java</code>, and <code>NovadTeleOp.java</code> to your TeamCode</p>
                        </div>
                    </div>
                    <div className="step">
                        <div className="step-number">2</div>
                        <div className="step-content">
                            <h3>Edit NovadSetup.java</h3>
                            <p>Fill in your motor names and pick your odometry type</p>
                        </div>
                    </div>
                    <div className="step">
                        <div className="step-number">3</div>
                        <div className="step-content">
                            <h3>Deploy & Drive</h3>
                            <p>Select "Novad TeleOp" and you're done!</p>
                        </div>
                    </div>
                </div>
                <div style={{ textAlign: 'center', marginTop: '2rem' }}>
                    <button className="btn btn-primary" onClick={() => setPage('setup')}>
                        See Full Setup Guide →
                    </button>
                </div>
            </section>
        </>
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP PAGE
// ═══════════════════════════════════════════════════════════════════════════

function SetupPage() {
    return (
        <section className="docs-section">
            <h1>Setup Guide</h1>
            
            <div className="card">
                <h2>Step 1: Copy Files</h2>
                <p>Copy these into your TeamCode folder:</p>
                <pre className="file-tree">{`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── NovadSetup.java     ← Configure here
├── NovadTeleOp.java    ← Ready to use
└── novad/              ← Don't edit this`}</pre>
            </div>

            <div className="card">
                <h2>Step 2: Edit NovadSetup.java</h2>
                <p>This is the <strong>only file you need to edit</strong>. Fill in your robot's values:</p>
                <CodeBlock code={setupCode} />
            </div>

            <div className="card">
                <h2>Step 3: Deploy & Run</h2>
                <CodeBlock code={teleOpCode} />
            </div>

            <div className="card">
                <h2>Troubleshooting</h2>
                <table className="troubleshoot-table">
                    <tbody>
                        <tr>
                            <td><strong>Robot drives backwards?</strong></td>
                            <td>Flip the <code>_REVERSED</code> values</td>
                        </tr>
                        <tr>
                            <td><strong>Doesn't resist pushes?</strong></td>
                            <td>Increase <code>POSITION_P</code></td>
                        </tr>
                        <tr>
                            <td><strong>Robot shakes/oscillates?</strong></td>
                            <td>Decrease <code>POSITION_P</code> or increase <code>POSITION_D</code></td>
                        </tr>
                    </tbody>
                </table>
            </div>
        </section>
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// TUNING PAGE
// ═══════════════════════════════════════════════════════════════════════════

function TuningPage() {
    return (
        <section className="docs-section">
            <h1>Tuning Guide</h1>
            
            <div className="card">
                <h2>Connect to FTC Dashboard</h2>
                <p>While your robot is running:</p>
                <ol>
                    <li>Connect to robot WiFi</li>
                    <li>Go to <a href="http://192.168.43.1:8080/dash" target="_blank">192.168.43.1:8080/dash</a></li>
                    <li>Or use <a href="https://panels.bylazar.com" target="_blank">panels.bylazar.com</a></li>
                    <li>Find <code>NovadSetup</code> in the sidebar</li>
                </ol>
            </div>

            <div className="card">
                <h2>What to Tune</h2>
                <CodeBlock code={tuningCode} />
            </div>

            <div className="card">
                <h2>Tuning Tips</h2>
                <div className="tip">
                    <h4>🎯 Position (Push Resistance)</h4>
                    <p>Start with <code>POSITION_P = 0.046</code>. Push the robot and see if it returns. Increase if too weak, decrease if it shakes.</p>
                </div>
                <div className="tip">
                    <h4>🔄 Heading (Rotation Resistance)</h4>
                    <p>Start with <code>HEADING_P = 0.67</code>. Try to rotate the robot. Increase if too weak.</p>
                </div>
                <div className="tip">
                    <h4>⚡ Predictive Defense</h4>
                    <p>Leave <code>PREDICTIVE_ENABLED = true</code>. This makes Novad respond ~50ms faster by predicting pushes.</p>
                </div>
            </div>
        </section>
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// APP
// ═══════════════════════════════════════════════════════════════════════════

function App() {
    const [currentPage, setPage] = useState('home');

    const handlePageChange = (page) => {
        setPage(page);
        window.scrollTo(0, 0);
    };

    return (
        <>
            <Navbar currentPage={currentPage} setPage={handlePageChange} />
            <main>
                {currentPage === 'home' && <HomePage setPage={handlePageChange} />}
                {currentPage === 'setup' && <SetupPage />}
                {currentPage === 'tuning' && <TuningPage />}
            </main>
            <footer className="footer">
                <p>🛡️ Novad — Built for FTC teams</p>
            </footer>
        </>
    );
}

ReactDOM.createRoot(document.getElementById('root')).render(<App />);
