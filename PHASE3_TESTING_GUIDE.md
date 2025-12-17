# Phase 3 Testing Guide - Verify All Functionality

This guide helps you verify that Phase 3 GitHub OAuth implementation is working correctly.

## Prerequisites

- Backend running: `python -m uvicorn src.main:app --reload` (port 8000)
- Frontend running: `npm run dev` (port 3000)
- GitHub OAuth app configured with credentials in `.env`
- API client (curl, Postman, or VS Code REST Client)

## Frontend Testing

### 1. Test Frontend Loads Without Errors

**Expected**: Frontend loads at http://localhost:3000 without errors

**Steps**:
1. Open http://localhost:3000 in browser
2. Open Developer Console (F12)
3. Check Console tab for errors

**Success Criteria**:
- ✅ No "process is not defined" errors
- ✅ No network errors
- ✅ Page loads and renders

**If Error**: Make sure all hooks are using the corrected API_URL pattern

---

### 2. Test GitHub Login Button Appears

**Expected**: GitHub "Sign in with GitHub" button visible in auth modal

**Steps**:
1. Look for login/signup button on homepage
2. Click to open authentication modal
3. Look for "Sign in with GitHub" option

**Success Criteria**:
- ✅ GitHub button appears with GitHub icon
- ✅ Email/password form also visible
- ✅ Can toggle between auth methods

---

### 3. Test OAuth Button Works

**Expected**: Clicking GitHub button redirects to GitHub login

**Steps**:
1. Click "Sign in with GitHub" button
2. You should be redirected to github.com/login

**Success Criteria**:
- ✅ Redirected to GitHub authorization page
- ✅ Page shows "Authorize [app name]"
- ✅ Request permissions: user:email

---

## Backend API Testing

### 1. Test GET /api/auth/github

**Purpose**: Initiate OAuth flow and get auth URL

**Command**:
```bash
curl -X GET http://localhost:8000/api/auth/github
```

**Expected Response** (200):
```json
{
  "auth_url": "https://github.com/login/oauth/authorize?client_id=...",
  "state": "random-state-token-here"
}
```

**Success Criteria**:
- ✅ Status code: 200
- ✅ `auth_url` contains "github.com"
- ✅ `state` is non-empty string
- ✅ Response is valid JSON

---

### 2. Test GET /api/auth/github/callback

**Purpose**: Handle OAuth callback and create user

**Note**: This endpoint requires a real GitHub authorization code. You can't test directly with curl.

**Manual Test**:
1. Complete GitHub login flow manually
2. Check browser DevTools Network tab
3. Look for request to `/api/auth/github/callback?code=...`

**Expected Response** (200):
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer",
  "user": {
    "id": "uuid-here",
    "email": "your@email.com",
    "hardware_background": "beginner",
    "software_background": "none"
  }
}
```

**Success Criteria**:
- ✅ Status code: 200
- ✅ Tokens returned are valid JWT format
- ✅ User email matches GitHub email
- ✅ Background fields set to defaults

---

### 3. Test POST /api/auth/background

**Purpose**: Update user expertise levels after OAuth

**Setup**: You need a valid JWT token from OAuth flow

**Command**:
```bash
curl -X POST http://localhost:8000/api/auth/background \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN_HERE" \
  -H "Content-Type: application/json" \
  -d '{
    "hardware_background": "intermediate",
    "software_background": "some_python"
  }'
```

**Expected Response** (200):
```json
{
  "id": "uuid-here",
  "email": "user@example.com",
  "hardware_background": "intermediate",
  "software_background": "some_python"
}
```

**Success Criteria**:
- ✅ Status code: 200
- ✅ Background fields updated correctly
- ✅ User ID matches authenticated user

---

### 4. Test GET /api/auth/me

**Purpose**: Get current authenticated user profile

**Command**:
```bash
curl -X GET http://localhost:8000/api/auth/me \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN_HERE"
```

**Expected Response** (200):
```json
{
  "id": "uuid-here",
  "email": "user@example.com",
  "hardware_background": "intermediate",
  "software_background": "some_python"
}
```

**Success Criteria**:
- ✅ Status code: 200
- ✅ Returns authenticated user's data
- ✅ All fields present

**Without Token**:
```bash
curl -X GET http://localhost:8000/api/auth/me
```

Expected: Status 403 (Forbidden) or 401 (Unauthorized)

---

### 5. Test POST /api/auth/refresh

**Purpose**: Refresh access token using refresh token

**Command**:
```bash
curl -X POST "http://localhost:8000/api/auth/refresh?refresh_token=YOUR_REFRESH_TOKEN_HERE"
```

**Expected Response** (200):
```json
{
  "access_token": "new-token-here",
  "token_type": "bearer"
}
```

**Success Criteria**:
- ✅ Status code: 200
- ✅ New access_token returned
- ✅ Token is valid JWT format

---

### 6. Test POST /api/auth/logout

**Purpose**: Logout authenticated user

**Command**:
```bash
curl -X POST http://localhost:8000/api/auth/logout \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN_HERE"
```

**Expected Response** (200):
```json
{
  "message": "Successfully logged out"
}
```

**Success Criteria**:
- ✅ Status code: 200
- ✅ Success message returned

---

## Running Backend Tests

### Run Unit Tests

```bash
cd backend
python -m pytest tests/test_auth_service.py -v
```

**Expected Output**:
```
tests/test_auth_service.py::TestAuthServiceTokens::test_generate_state_token PASSED
tests/test_auth_service.py::TestAuthServiceTokens::test_create_jwt_token PASSED
tests/test_auth_service.py::TestAuthServiceTokens::test_verify_jwt_token_valid PASSED
...
```

**Success Criteria**:
- ✅ All 15 tests PASS
- ✅ No errors or warnings

### Run Integration Tests

```bash
cd backend
python -m pytest tests/test_oauth_integration.py -v
```

**Expected Output**:
```
tests/test_oauth_integration.py::TestGitHubOAuthEndpoints::test_github_oauth_login_endpoint PASSED
tests/test_oauth_integration.py::TestGitHubOAuthEndpoints::test_github_oauth_callback_success PASSED
...
```

**Success Criteria**:
- ✅ All 10 tests PASS
- ✅ No errors or warnings

### Run All Tests with Coverage

```bash
cd backend
python -m pytest tests/ --cov=src/services --cov=src/api/routers -v
```

**Success Criteria**:
- ✅ 25+ tests PASS
- ✅ Coverage > 80%

---

## End-to-End Flow Test

### Complete OAuth Flow

1. **Start Frontend**:
   ```bash
   cd docusaurus
   npm run dev
   ```

2. **Start Backend**:
   ```bash
   cd backend
   python -m uvicorn src.main:app --reload
   ```

3. **Open Browser**:
   - Visit http://localhost:3000
   - Open Developer Console (F12)

4. **Click GitHub Button**:
   - Click "Sign in with GitHub"
   - You'll be redirected to GitHub

5. **Authorize App**:
   - Log in to GitHub if needed
   - Click "Authorize" button
   - Accept required permissions

6. **Verify Callback**:
   - Check Console for errors
   - You should see the background expertise form
   - Verify network request to `/api/auth/github/callback`

7. **Set Background**:
   - Select hardware background
   - Select software background
   - Click "Save & Continue"

8. **Verify Success**:
   - Check localStorage for tokens
   - Should be redirected to home page
   - User should be logged in

**Success Criteria**:
- ✅ No console errors
- ✅ OAuth callback succeeds
- ✅ Background form displays
- ✅ Tokens saved in localStorage
- ✅ User logged in on home page

---

## Troubleshooting

### Error: "process is not defined"

**Cause**: Browser can't access process.env

**Fix**: Already applied in all hook files. Make sure you're using:
```typescript
const API_URL = typeof window !== 'undefined'
  ? (window as any).__API_URL || 'http://localhost:8000'
  : 'http://localhost:8000';
```

**Verify**: Run frontend again after fix

---

### Error: "GitHub authentication failed"

**Cause**: OAuth configuration issue

**Check**:
1. GitHub OAuth app credentials in `.env`
2. Redirect URI matches in GitHub app settings
3. Backend is running on correct port
4. Network connectivity to github.com

**Fix**:
1. Verify `.env` has correct `GITHUB_OAUTH_CLIENT_ID` and `GITHUB_OAUTH_CLIENT_SECRET`
2. Check GitHub app settings at https://github.com/settings/developers
3. Ensure callback URL is: `http://localhost:8000/api/auth/github/callback`

---

### Error: "Invalid state token"

**Cause**: CSRF protection failed

**Check**:
1. Browser localStorage is enabled
2. SessionStorage is enabled
3. No browser cache interference

**Fix**:
1. Clear browser cache
2. Check browser privacy settings
3. Test in incognito/private mode

---

### Error: "Token expired"

**Cause**: Access token lifetime exceeded

**Expected Behavior**: Frontend should auto-refresh using refresh token

**Verify**:
1. Check localStorage for `refresh_token`
2. Next API call should use new token
3. Should not require re-login

---

## Performance Testing

### Check Token Generation Speed

```bash
# Time a token generation request
time curl -X GET http://localhost:8000/api/auth/github
```

**Expected**: < 100ms

### Check Auth Endpoint Response Time

```bash
# Profile OAuth callback (requires manual OAuth flow)
# Check DevTools Network tab -> /api/auth/github/callback
```

**Expected**: < 500ms

---

## Security Verification

### Check CSRF Protection

1. Complete OAuth flow
2. Check Network tab -> `/api/auth/github/callback`
3. Verify `state` parameter is included
4. Verify `state` matches request

### Check Token Format

```bash
# Decode JWT token (don't verify, just inspect)
# Use https://jwt.io and paste your token

# Expected payload:
{
  "sub": "user-id-uuid",
  "exp": 1702827600,
  "iat": 1702824000,
  "type": "access"
}
```

### Check Bearer Token Usage

```bash
# Valid request with Bearer token
curl -X GET http://localhost:8000/api/auth/me \
  -H "Authorization: Bearer YOUR_TOKEN"

# Invalid request without token
curl -X GET http://localhost:8000/api/auth/me
# Should return 403/401
```

---

## Browser DevTools Verification

### Check localStorage

Open DevTools → Application → Local Storage → http://localhost:3000

You should see:
- ✅ `access_token` - JWT token
- ✅ `refresh_token` - JWT refresh token

### Check Network Requests

Open DevTools → Network tab

When clicking GitHub button:
1. ✅ GET /api/auth/github - State generation
2. ✅ Redirect to github.com
3. ✅ GET /api/auth/github/callback - Token exchange
4. ✅ Redirect to /oauth-callback - Callback handling

### Check Console

Look for:
- ✅ No "process is not defined" errors
- ✅ No 401/403 auth errors
- ✅ No CORS errors

---

## Verification Checklist

Before considering Phase 3 complete:

- [ ] Frontend loads without errors
- [ ] GitHub OAuth button appears
- [ ] OAuth redirects to GitHub work
- [ ] API endpoints respond with 200
- [ ] Tokens are generated and stored
- [ ] Background expertise form appears
- [ ] User can update background
- [ ] Tokens are valid JWT format
- [ ] All 25 tests pass
- [ ] No console errors
- [ ] CSRF protection working
- [ ] Token refresh works
- [ ] Protected routes require authentication
- [ ] Logout clears tokens

---

## Quick Test Commands

```bash
# 1. Test OAuth init endpoint
curl http://localhost:8000/api/auth/github | jq

# 2. Run unit tests
cd backend && pytest tests/test_auth_service.py -v

# 3. Run integration tests
cd backend && pytest tests/test_oauth_integration.py -v

# 4. Check frontend loads
curl http://localhost:3000 | head -20

# 5. Get current user (with token)
curl -H "Authorization: Bearer TOKEN" http://localhost:8000/api/auth/me | jq
```

---

## Support

If tests fail or endpoints don't work:

1. Check `.env` file has all required variables
2. Verify backend is running (`ps aux | grep uvicorn`)
3. Verify frontend is running (`ps aux | grep npm`)
4. Check for error messages in console/logs
5. Review PHASE3_QUICK_START.md for setup instructions
6. Review PHASE3_COMPLETION_SUMMARY.md for detailed specs

---

**Last Updated**: December 17, 2024
**Status**: All tests written and ready for verification
