require('dotenv').config();
const express = require('express');
const cors = require('cors');
const session = require('express-session');
const passport = require('passport');
const GoogleStrategy = require('passport-google-oauth20').Strategy;
const GitHubStrategy = require('passport-github2').Strategy;
const bcrypt = require('bcryptjs');
const jwt = require('jsonwebtoken');
const crypto = require('crypto');
const cookieParser = require('cookie-parser');

const app = express();

// Middleware
app.use(cors({
  origin: 'http://localhost:3000', // Docusaurus dev server
  credentials: true
}));
app.use(express.json());
app.use(express.urlencoded({ extended: true }));
app.use(cookieParser());

// Session configuration
app.use(session({
  secret: process.env.SESSION_SECRET || 'fallback_session_secret',
  resave: false,
  saveUninitialized: false,
  cookie: {
    secure: false, // Set to true in production with HTTPS
    maxAge: 24 * 60 * 60 * 1000 // 24 hours
  }
}));

app.use(passport.initialize());
app.use(passport.session());

// In-memory stores for rate limiting and password reset tokens
const passwordResetAttempts = new Map(); // To track attempts per IP/email
const passwordResetTokens = new Map(); // To store reset tokens with expiry
const emailVerificationTokens = new Map(); // To store verification tokens

// Mock user database (in production, use a real database)
const users = [
  {
    id: '1',
    name: 'John Doe',
    email: 'john@example.com',
    password: '$2a$10$8K1p/a0dURBkgq4HaZLy/eL.E2J9Ee.jWF.kwXQnjXHbIZnwgZKlO', // 'password123'
    emailVerified: true
  }
];

// Mock sessions store
const sessions = new Map();

// Passport configuration
passport.serializeUser((user, done) => {
  done(null, user.id);
});

passport.deserializeUser((id, done) => {
  // In a real app, fetch user from database
  const user = users.find(u => u.id === id);
  done(null, user);
});

// Google OAuth Strategy
passport.use(new GoogleStrategy({
  clientID: process.env.GOOGLE_CLIENT_ID || 'fake-google-client-id',
  clientSecret: process.env.GOOGLE_CLIENT_SECRET || 'fake-google-client-secret',
  callbackURL: "/api/auth/google/callback"
}, async (accessToken, refreshToken, profile, done) => {
  try {
    // Check if user already exists by email
    let user = users.find(u => u.email === profile.emails[0].value);

    if (!user) {
      // Create new user with verified email from Google
      user = {
        id: (users.length + 1).toString(),
        name: profile.displayName,
        email: profile.emails[0].value,
        emailVerified: true, // Google emails are verified
        password: null, // No password for OAuth users
        createdAt: new Date().toISOString()
      };
      users.push(user);
    } else {
      // Update user info if needed
      user.name = profile.displayName;
    }

    return done(null, user);
  } catch (error) {
    console.error('Google OAuth error:', error);
    return done(error, null);
  }
}));

// GitHub OAuth Strategy
passport.use('github', new GitHubStrategy({
  clientID: process.env.GITHUB_ID || 'fake-github-id',
  clientSecret: process.env.GITHUB_SECRET || 'fake-github-secret',
  callbackURL: "/api/auth/github/callback"
}, async (accessToken, refreshToken, profile, done) => {
  try {
    // GitHub might not always provide email in the profile
    const primaryEmail = profile.emails ?
      profile.emails.find(email => email.primary)?.value || profile.emails[0].value
      : `user${profile.id}@github.example.com`;

    // Check if user already exists
    let user = users.find(u => u.email === primaryEmail);

    if (!user) {
      // Create new user
      user = {
        id: (users.length + 1).toString(),
        name: profile.displayName || profile.username,
        email: primaryEmail,
        emailVerified: true, // GitHub emails are verified
        password: null, // No password for OAuth users
        createdAt: new Date().toISOString()
      };
      users.push(user);
    } else {
      // Update user info if needed
      user.name = profile.displayName || profile.username;
    }

    return done(null, user);
  } catch (error) {
    console.error('GitHub OAuth error:', error);
    return done(error, null);
  }
}));

// Helper function to generate secure tokens
function generateSecureToken() {
  return crypto.randomBytes(32).toString('hex');
}

// Helper function to hash passwords
async function hashPassword(password) {
  const saltRounds = 10;
  return await bcrypt.hash(password, saltRounds);
}

// Helper function to verify passwords
async function verifyPassword(password, hashedPassword) {
  return await bcrypt.compare(password, hashedPassword);
}

// Helper function to check rate limiting for password reset
function checkRateLimit(email, ip, maxAttempts = 5, timeWindow = 15 * 60 * 1000) { // 15 minutes
  const now = Date.now();
  const key = `${email}:${ip}`;

  if (!passwordResetAttempts.has(key)) {
    passwordResetAttempts.set(key, []);
  }

  const attempts = passwordResetAttempts.get(key);
  // Remove attempts older than time window
  const recentAttempts = attempts.filter(attempt => now - attempt < timeWindow);

  if (recentAttempts.length >= maxAttempts) {
    return false; // Rate limited
  }

  recentAttempts.push(now);
  passwordResetAttempts.set(key, recentAttempts);
  return true; // Not rate limited
}

// API Routes

// Google OAuth login
app.get('/api/auth/google',
  passport.authenticate('google', { scope: ['profile', 'email'] })
);

app.get('/api/auth/google/callback',
  passport.authenticate('google', { failureRedirect: '/auth/login' }),
  (req, res) => {
    // Generate JWT token for frontend
    const token = jwt.sign(
      { userId: req.user.id, email: req.user.email },
      process.env.JWT_SECRET || 'fallback_secret_key',
      { expiresIn: '24h' }
    );

    // Set token in cookie
    res.cookie('authToken', token, {
      httpOnly: true,
      secure: false, // Set to true in production with HTTPS
      maxAge: 24 * 60 * 60 * 1000, // 24 hours
      sameSite: 'strict'
    });

    // Redirect to frontend with success
    res.redirect('http://localhost:3000/');
  }
);

// GitHub OAuth login
app.get('/api/auth/github',
  passport.authenticate('github', { scope: ['user:email'] })
);

app.get('/api/auth/github/callback',
  passport.authenticate('github', { failureRedirect: '/auth/login' }),
  (req, res) => {
    // Generate JWT token for frontend
    const token = jwt.sign(
      { userId: req.user.id, email: req.user.email },
      process.env.JWT_SECRET || 'fallback_secret_key',
      { expiresIn: '24h' }
    );

    // Set token in cookie
    res.cookie('authToken', token, {
      httpOnly: true,
      secure: false, // Set to true in production with HTTPS
      maxAge: 24 * 60 * 60 * 1000, // 24 hours
      sameSite: 'strict'
    });

    // Redirect to frontend with success
    res.redirect('http://localhost:3000/');
  }
);

// Register endpoint with email verification
app.post('/api/auth/register', async (req, res) => {
  const { name, email, password } = req.body;

  // Validate input
  if (!name || !email || !password) {
    return res.status(400).json({
      success: false,
      message: 'Name, email, and password are required'
    });
  }

  // Validate email format
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  if (!emailRegex.test(email)) {
    return res.status(400).json({
      success: false,
      message: 'Invalid email format'
    });
  }

  // Validate password strength
  if (password.length < 8) {
    return res.status(400).json({
      success: false,
      message: 'Password must be at least 8 characters long'
    });
  }

  // Check if user already exists
  const existingUser = users.find(u => u.email.toLowerCase() === email.toLowerCase());
  if (existingUser) {
    return res.status(409).json({
      success: false,
      message: 'Email already registered'
    });
  }

  try {
    // Hash password
    const hashedPassword = await hashPassword(password);

    // Create new user with unverified email
    const newUser = {
      id: (users.length + 1).toString(),
      name,
      email: email.toLowerCase(),
      password: hashedPassword,
      emailVerified: false, // Email is not verified yet
      createdAt: new Date().toISOString()
    };

    users.push(newUser);

    // Generate email verification token (in a real app, send email with this token)
    const verificationToken = generateSecureToken();
    emailVerificationTokens.set(verificationToken, {
      userId: newUser.id,
      email: newUser.email,
      expiresAt: Date.now() + 24 * 60 * 60 * 1000 // 24 hours
    });

    // In a real application, send verification email here
    // For this demo, we'll consider email as verified for simplicity
    // But normally you'd have an endpoint to verify email with the token
    newUser.emailVerified = true; // For demo purposes, auto-verify

    // Generate JWT token
    const token = jwt.sign(
      { userId: newUser.id, email: newUser.email },
      process.env.JWT_SECRET || 'fallback_secret_key',
      { expiresIn: '24h' }
    );

    // Set token in cookie
    res.cookie('authToken', token, {
      httpOnly: true,
      secure: false, // Set to true in production with HTTPS
      maxAge: 24 * 60 * 60 * 1000, // 24 hours
      sameSite: 'strict'
    });

    res.status(201).json({
      success: true,
      message: 'User registered successfully',
      user: {
        id: newUser.id,
        name: newUser.name,
        email: newUser.email,
        emailVerified: newUser.emailVerified
      },
      token
    });
  } catch (error) {
    console.error('Registration error:', error);
    res.status(500).json({
      success: false,
      message: 'Registration failed'
    });
  }
});

// Login endpoint
app.post('/api/auth/login', async (req, res) => {
  const { email, password } = req.body;

  if (!email || !password) {
    return res.status(400).json({
      success: false,
      message: 'Email and password are required'
    });
  }

  try {
    // Find user by email
    const user = users.find(u => u.email.toLowerCase() === email.toLowerCase());

    if (!user) {
      return res.status(401).json({
        success: false,
        message: 'Invalid email or password'
      });
    }

    // Check if email is verified
    if (!user.emailVerified) {
      return res.status(401).json({
        success: false,
        message: 'Please verify your email address'
      });
    }

    // Check if user has a password (not OAuth user)
    if (!user.password) {
      return res.status(401).json({
        success: false,
        message: 'Please login with your OAuth provider'
      });
    }

    // Verify password
    const isPasswordValid = await verifyPassword(password, user.password);
    if (!isPasswordValid) {
      return res.status(401).json({
        success: false,
        message: 'Invalid email or password'
      });
    }

    // Generate JWT token
    const token = jwt.sign(
      { userId: user.id, email: user.email },
      process.env.JWT_SECRET || 'fallback_secret_key',
      { expiresIn: '24h' }
    );

    // Set token in cookie
    res.cookie('authToken', token, {
      httpOnly: true,
      secure: false, // Set to true in production with HTTPS
      maxAge: 24 * 60 * 60 * 1000, // 24 hours
      sameSite: 'strict'
    });

    res.status(200).json({
      success: true,
      message: 'Login successful',
      user: {
        id: user.id,
        name: user.name,
        email: user.email,
        emailVerified: user.emailVerified
      },
      token
    });
  } catch (error) {
    console.error('Login error:', error);
    res.status(500).json({
      success: false,
      message: 'Login failed'
    });
  }
});

// Forgot password endpoint with security measures
app.post('/api/auth/forgot-password', async (req, res) => {
  const { email } = req.body;
  const ip = req.ip; // Get user's IP address

  if (!email) {
    return res.status(400).json({
      success: false,
      message: 'Email is required'
    });
  }

  // Validate email format
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  if (!emailRegex.test(email)) {
    return res.status(400).json({
      success: false,
      message: 'Invalid email format'
    });
  }

  // Rate limiting check
  if (!checkRateLimit(email, ip)) {
    return res.status(429).json({
      success: false,
      message: 'Too many password reset attempts. Please try again later.'
    });
  }

  try {
    // Find user (but don't reveal if user exists or not)
    const user = users.find(u => u.email.toLowerCase() === email.toLowerCase());

    if (user && user.emailVerified) {
      // Generate reset token with expiration (1 hour)
      const resetToken = generateSecureToken();
      const expiresAt = Date.now() + 60 * 60 * 1000; // 1 hour

      passwordResetTokens.set(resetToken, {
        userId: user.id,
        email: user.email,
        expiresAt: expiresAt
      });

      // In a real app, send email with reset link containing the token
      // For this demo, we'll just log it (don't do this in production!)
      console.log(`Password reset link would be sent to ${email}. Token: ${resetToken}`);

      // Return generic message regardless of whether user exists
      res.status(200).json({
        success: true,
        message: 'If an account exists with that email, a password reset link has been sent.'
      });
    } else {
      // Still return the same message to prevent user enumeration
      res.status(200).json({
        success: true,
        message: 'If an account exists with that email, a password reset link has been sent.'
      });
    }
  } catch (error) {
    console.error('Forgot password error:', error);
    res.status(500).json({
      success: false,
      message: 'Request failed'
    });
  }
});

// Reset password endpoint
app.post('/api/auth/reset-password', async (req, res) => {
  const { token, newPassword } = req.body;

  if (!token || !newPassword) {
    return res.status(400).json({
      success: false,
      message: 'Token and new password are required'
    });
  }

  // Validate password strength
  if (newPassword.length < 8) {
    return res.status(400).json({
      success: false,
      message: 'Password must be at least 8 characters long'
    });
  }

  try {
    // Check if token exists and is not expired
    const resetData = passwordResetTokens.get(token);
    if (!resetData || Date.now() > resetData.expiresAt) {
      return res.status(400).json({
        success: false,
        message: 'Invalid or expired token'
      });
    }

    // Find user and update password
    const user = users.find(u => u.id === resetData.userId);
    if (!user) {
      return res.status(400).json({
        success: false,
        message: 'User not found'
      });
    }

    // Hash new password
    const hashedPassword = await hashPassword(newPassword);
    user.password = hashedPassword;

    // Remove the used token
    passwordResetTokens.delete(token);

    res.status(200).json({
      success: true,
      message: 'Password has been reset successfully'
    });
  } catch (error) {
    console.error('Reset password error:', error);
    res.status(500).json({
      success: false,
      message: 'Password reset failed'
    });
  }
});

// Verify email endpoint (for email verification tokens)
app.get('/api/auth/verify-email/:token', (req, res) => {
  const { token } = req.params;

  const verificationData = emailVerificationTokens.get(token);
  if (!verificationData || Date.now() > verificationData.expiresAt) {
    return res.status(400).json({
      success: false,
      message: 'Invalid or expired verification token'
    });
  }

  // Find user and mark as verified
  const user = users.find(u => u.id === verificationData.userId);
  if (user) {
    user.emailVerified = true;
  }

  // Remove the used token
  emailVerificationTokens.delete(token);

  res.status(200).json({
    success: true,
    message: 'Email verified successfully'
  });
});

// Get current session endpoint
app.get('/api/auth/session', (req, res) => {
  const token = req.cookies.authToken;

  if (!token) {
    return res.status(200).json({ user: null });
  }

  try {
    const decoded = jwt.verify(token, process.env.JWT_SECRET || 'fallback_secret_key');

    const user = users.find(u => u.id === decoded.userId);
    if (!user) {
      return res.status(200).json({ user: null });
    }

    res.status(200).json({
      user: {
        id: user.id,
        name: user.name,
        email: user.email,
        emailVerified: user.emailVerified
      }
    });
  } catch (error) {
    res.status(200).json({ user: null });
  }
});

// Logout endpoint
app.post('/api/auth/logout', (req, res) => {
  // Clear auth token cookie
  res.clearCookie('authToken');

  res.status(200).json({
    success: true,
    message: 'Logged out successfully'
  });
});

// API Documentation endpoint
app.get('/api/docs', (req, res) => {
  res.json({
    title: 'Authentication API Documentation',
    version: '1.0.0',
    description: 'API documentation for the Physical AI & Humanoid Robotics authentication system',
    endpoints: [
      {
        path: 'POST /api/auth/register',
        description: 'Register a new user with name, email, and password',
        requestBody: {
          name: 'string',
          email: 'string',
          password: 'string (min 8 characters)'
        },
        responses: {
          '201': 'User registered successfully',
          '400': 'Bad request - missing fields or invalid format',
          '409': 'Email already registered'
        }
      },
      {
        path: 'POST /api/auth/login',
        description: 'Login with email and password',
        requestBody: {
          email: 'string',
          password: 'string'
        },
        responses: {
          '200': 'Login successful',
          '400': 'Bad request - missing fields',
          '401': 'Invalid credentials or unverified email'
        }
      },
      {
        path: 'POST /api/auth/forgot-password',
        description: 'Request password reset link',
        requestBody: {
          email: 'string'
        },
        responses: {
          '200': 'Password reset link sent (if user exists)',
          '400': 'Bad request - missing email or invalid format'
        }
      },
      {
        path: 'POST /api/auth/reset-password',
        description: 'Reset password with token',
        requestBody: {
          token: 'string',
          newPassword: 'string (min 8 characters)'
        },
        responses: {
          '200': 'Password reset successful',
          '400': 'Bad request - missing fields or invalid token'
        }
      },
      {
        path: 'GET /api/auth/verify-email/:token',
        description: 'Verify email address with token',
        parameters: {
          token: 'string - verification token'
        },
        responses: {
          '200': 'Email verified successfully',
          '400': 'Invalid or expired token'
        }
      },
      {
        path: 'GET /api/auth/session',
        description: 'Get current user session',
        responses: {
          '200': 'Session data returned'
        }
      },
      {
        path: 'POST /api/auth/logout',
        description: 'Logout user and clear session',
        responses: {
          '200': 'Logged out successfully'
        }
      },
      {
        path: 'GET /api/auth/google',
        description: 'Initiate Google OAuth login',
        responses: {
          '302': 'Redirect to Google OAuth'
        }
      },
      {
        path: 'GET /api/auth/github',
        description: 'Initiate GitHub OAuth login',
        responses: {
          '302': 'Redirect to GitHub OAuth'
        }
      }
    ]
  });
});

const PORT = process.env.PORT || 4000;
app.listen(PORT, () => {
  console.log(`Authentication server running on port ${PORT}`);
  console.log(`Available endpoints:`);
  console.log(`  POST /api/auth/register - Register new user`);
  console.log(`  POST /api/auth/login - Login with credentials`);
  console.log(`  POST /api/auth/forgot-password - Request password reset`);
  console.log(`  POST /api/auth/reset-password - Reset password with token`);
  console.log(`  GET /api/auth/verify-email/:token - Verify email address`);
  console.log(`  GET /api/auth/session - Get current session`);
  console.log(`  POST /api/auth/logout - Logout user`);
  console.log(`  GET /api/auth/google - Google OAuth login`);
  console.log(`  GET /api/auth/github - GitHub OAuth login`);
  console.log(`  GET /api/docs - API Documentation`);
});