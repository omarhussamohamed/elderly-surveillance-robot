# Frontend - Elderly Surveillance Robot Mobile App

Flutter mobile application for monitoring and controlling the Elderly Surveillance Robot system. Provides real-time monitoring, remote control, emergency alerts, and user management.

---

## Overview

The mobile app serves as the primary interface for caregivers and family members to interact with the elderly surveillance robot. It connects to the FastAPI backend to authenticate users, retrieve telemetry data, and manage robot operations.

**Key Features:**
- User authentication (login/registration)
- Real-time system health monitoring (battery, temperature)
- Safety status monitoring (gas, fire, fall, stranger detection)
- Remote robot control
- Emergency call functionality
- Profile and caregiver management
- Notification system

---

## Technology Stack

- **Framework:** Flutter 3.x
- **Language:** Dart SDK ^3.7.0
- **State Management:** flutter_bloc (BLoC pattern)
- **HTTP Client:** dio (^5.9.0)
- **Local Storage:** shared_preferences (^2.5.3)
- **Vector Graphics:** flutter_svg (^2.2.2)
- **Icons:** cupertino_icons (^1.0.8)
- **State Utilities:** equatable (^2.0.8)

---

## Folder Structure

```
lib/
├── main.dart                    # App entry point with BLoC providers
├── welcome_screen.dart          # Initial landing screen
├── core/                        # Core utilities and configuration
│   ├── const.dart              # App constants (colors, sizes, routes)
│   ├── network/                # HTTP client configuration
│   │   └── dio_client.dart
│   └── on_generate_route.dart  # App navigation routing
└── features/                    # Feature modules (clean architecture)
    ├── authentication/          # User login and registration
    │   ├── data/
    │   │   ├── datasources/
    │   │   ├── models/
    │   │   └── repositories/
    │   ├── domain/
    │   │   ├── entities/
    │   │   ├── repositories/
    │   │   └── usecases/
    │   └── presentation/
    │       ├── cubit/
    │       └── screens/
    ├── home/                    # Main dashboard and monitoring
    │   ├── data/
    │   ├── domain/
    │   └── presentation/
    │       ├── cubit/
    │       └── screens/
    │           ├── home_screen.dart
    │           ├── remote_screen.dart
    │           └── menu_screen.dart
    ├── main_screen/            # Bottom navigation container
    │   └── main_screen.dart
    ├── emergency/              # Emergency call feature
    ├── notifications/          # Alert notifications
    └── profile/                # User profile management
        └── presentation/
            └── screens/
                ├── profile_screen.dart
                ├── manage_people_screen.dart
                └── help_support_screen.dart
```

---

## Prerequisites

Before running the app, ensure you have:

- **Flutter SDK 3.7.0 or higher**
- **Dart SDK 3.7.0 or higher**
- **Android Studio** or **VS Code** with Flutter extensions
- **Android device/emulator** or **iOS device/simulator**
- **Backend server** running (see backend README)

---

## Installation

### 1. Install Flutter

Follow the official guide: https://docs.flutter.dev/get-started/install

Verify installation:

```bash
flutter doctor
```

### 2. Navigate to Frontend Directory

```bash
cd mobile_app/frontend
```

### 3. Install Dependencies

```bash
flutter pub get
```

### 4. Configure Backend Connection

Update the backend URL in the Dio client configuration:

**File:** `lib/core/network/dio_client.dart`

```dart
class DioClient {
  static Dio dio = Dio(
    BaseOptions(
      baseUrl: 'http://YOUR_BACKEND_IP:8000',  // Update this
      // For local testing: 'http://10.0.2.2:8000' (Android emulator)
      // For production: 'https://your-domain.com'
      connectTimeout: const Duration(seconds: 10),
      receiveTimeout: const Duration(seconds: 10),
      headers: {
        'Content-Type': 'application/json',
      },
    ),
  );
}
```

**Common Backend URLs:**
- Local development: `http://localhost:8000`
- Android emulator: `http://10.0.2.2:8000`
- iOS simulator: `http://localhost:8000`
- Production: `https://your-api-domain.com`

---

## Running the App

### Development Mode

```bash
flutter run
```

Select target device when prompted.

### Run on Specific Device

```bash
# List available devices
flutter devices

# Run on specific device
flutter run -d <device-id>
```

### Hot Reload

While app is running, press:
- `r` - Hot reload
- `R` - Hot restart
- `q` - Quit

---

## Building for Production

### Android APK

```bash
flutter build apk --release
```

Output: `build/app/outputs/flutter-apk/app-release.apk`

### Android App Bundle (Google Play)

```bash
flutter build appbundle --release
```

### iOS

```bash
flutter build ios --release
```

**Note:** Requires Mac with Xcode for iOS builds.

---

## Architecture

### Clean Architecture Layers

The app follows clean architecture principles with three layers:

1. **Presentation Layer**
   - UI screens (widgets)
   - BLoC (Business Logic Component)
   - State management

2. **Domain Layer**
   - Entities (business models)
   - Use cases (business logic)
   - Repository interfaces

3. **Data Layer**
   - Repository implementations
   - Data sources (remote API)
   - Data models

### State Management (BLoC Pattern)

**Example: Authentication Flow**

```
User Action → AuthCubit.signIn() → SignInUseCase
    ↓
Repository → RemoteDataSource → Backend API
    ↓
Response → AuthState (Loading/Success/Error)
    ↓
UI Update (BlocConsumer listens to state changes)
```

### Navigation

Routes are centrally defined in `core/on_generate_route.dart`:

```dart
ScreenConst.homeScreen          → HomeScreen
ScreenConst.mainScreen          → MainScreen (with bottom nav)
ScreenConst.signInSignUpScreen  → SignInSignUpScreen
ScreenConst.emergencyCallScreen → EmergencyCall
ScreenConst.profileScreen       → ProfileScreen
ScreenConst.notificationScreen  → NotificationScreen
```

---

## Key Features Walkthrough

### 1. Authentication

**Screens:**
- `welcome_screen.dart` - Initial screen with sign in/up buttons
- `sign_in_sign_up_screen.dart` - Login and registration form

**Flow:**
1. User enters phone number and password
2. App calls `/auth/login` or `/auth/register`
3. Backend returns JWT token
4. Token stored in SharedPreferences
5. Navigate to main screen

### 2. Main Dashboard

**Screen:** `main_screen.dart`

Bottom navigation with three tabs:
- **Home** - System health and safety monitoring
- **Remote** - Camera feed and robot controls
- **Menu** - Profile, settings, logout

### 3. System Monitoring

**Screen:** `home_screen.dart`

Displays:
- Connection status (CONNECTED/DISCONNECTED)
- Battery percentage
- Temperature
- Safety monitor cards:
  - Gas detection (Normal/Warning)
  - Fire detection (None/Detected)
  - Fall detection (None/Detected)
  - Stranger detection (None/Detected)
- Last event log
- Emergency call button

**Data Source:** 
- `/health` endpoint for battery and temperature
- `/safety` endpoint for safety status

### 4. Remote Control

**Screen:** `remote_screen.dart`

Features:
- Live camera feed display
- Robot action buttons:
  - Move to location
  - Sound alarm
  - Restart system
  - Sleep mode

### 5. Profile Management

**Screens:**
- `profile_screen.dart` - View elderly information
- `manage_people_screen.dart` - Add/remove caregivers
- `help_support_screen.dart` - User guide and support

---

## API Integration

### Authentication

**Endpoints Used:**
- `POST /auth/register` - Create new user
- `POST /auth/login` - User login

**Authentication Flow:**
1. User credentials sent to backend
2. Backend validates and returns JWT token
3. Token stored locally with SharedPreferences
4. Token included in Authorization header for protected requests

### Protected Endpoints

All system data endpoints require authentication:

```dart
Options(
  headers: {'Authorization': 'Bearer $token'}
)
```

**Endpoints:**
- `GET /health` - Get system health
- `GET /safety` - Get safety status
- `GET /telemetry` - Get real-time telemetry

---

## Assets Configuration

### Fonts

**Montserrat Regular (400)** is configured in `pubspec.yaml`:

```yaml
fonts:
  - family: Montserrat
    fonts:
      - asset: assets/fonts/Montserrat/static/Montserrat-Regular.ttf
        weight: 400
```

### Images and Icons

All assets are located in `assets/` directory:

```
assets/
├── camera_view.webp       # Camera feed placeholder
├── fall.svg               # Fall detection icon
├── fire.svg               # Fire detection icon
├── gas.svg                # Gas detection icon
├── stranger.svg           # Stranger detection icon
├── fonts/
│   └── Montserrat/
└── images/
    ├── home_icon/
    │   └── home.svg
    ├── link_logo/
    │   ├── link_logo.svg
    │   └── link_logo.png
    └── notification_icon/
        └── notification.svg
```

### Using Assets in Code

**SVG:**
```dart
SvgPicture.asset(
  'assets/fire.svg',
  width: 20,
  height: 20,
)
```

**Images:**
```dart
Image.asset(
  'assets/camera_view.webp',
  fit: BoxFit.cover,
)
```

---

## Theme and Styling

### Color Palette

Defined in `core/const.dart`:

```dart
AppColors.darkNavy    // Background
AppColors.lightNavy   // Card backgrounds
AppColors.white       // Text
AppColors.grey        // Secondary text
AppColors.green       // Success/safe status
AppColors.blue        // Primary actions
AppColors.red         // Danger/emergency
```

### Responsive Sizing

Helper functions in `core/const.dart`:

```dart
AppSizes.screenWidth(context)
AppSizes.screenHeight(context)
```

---

## Troubleshooting

### Backend Connection Failed

**Error:** `DioException: Connection refused`

**Solution:**
- Verify backend server is running
- Check `baseUrl` in `dio_client.dart`
- For Android emulator, use `http://10.0.2.2:8000`
- Ensure phone and backend on same network (for physical device)

### Assets Not Loading

**Error:** `Unable to load asset`

**Solution:**
- Verify asset paths in `pubspec.yaml`
- Run `flutter clean` then `flutter pub get`
- Check file exists in `assets/` directory
- Ensure correct file extension (.svg, .png, etc.)

### SVG Not Rendering

**Error:** SVG appears blank

**Solution:**
- Use `SvgPicture.asset()` not `Image.asset()` for .svg files
- Verify `flutter_svg` package is installed
- Check SVG file is valid

### Authentication Token Issues

**Error:** `401 Unauthorized`

**Solution:**
- Check token is stored correctly
- Verify token is not expired
- Ensure Authorization header format: `Bearer <token>`
- Clear app data and re-login

### Build Fails

**Error:** `Gradle build failed`

**Solution:**
```bash
cd android
./gradlew clean
cd ..
flutter clean
flutter pub get
flutter run
```

---

## Testing

### Run All Tests

```bash
flutter test
```

### Run Specific Test

```bash
flutter test test/widget_test.dart
```

### Integration Tests

```bash
flutter drive --target=test_driver/app.dart
```

---

## Code Quality

### Linting

Linting rules are configured in `analysis_options.yaml`.

Run linter:

```bash
flutter analyze
```

### Formatting

Format code:

```bash
dart format lib/
```

---

## Common Development Tasks

### Add New Dependency

1. Add to `pubspec.yaml`:
```yaml
dependencies:
  new_package: ^1.0.0
```

2. Install:
```bash
flutter pub get
```

### Create New Screen

1. Create file in appropriate feature folder
2. Add route in `on_generate_route.dart`
3. Add route constant in `core/const.dart`

### Add New API Endpoint

1. Update data source interface
2. Implement in data source implementation
3. Update repository
4. Create/update use case
5. Update BLoC/Cubit

---

## Performance Tips

1. **Use `const` constructors** where possible
2. **Avoid rebuilding entire widget tree** - use BLoC selectors
3. **Optimize images** - use appropriate formats and sizes
4. **Lazy load data** - paginate large lists
5. **Cache API responses** - use shared_preferences or local DB

---

## Security Best Practices

1. **Never hardcode credentials** in source code
2. **Use HTTPS** for production backend
3. **Validate user input** before sending to API
4. **Handle tokens securely** - use secure storage for production
5. **Implement proper error handling** - don't expose sensitive info

---

## Authors

**Maryse Hani** - Mobile App Development

**Mariam Waleed** - Mobile App Development

---

## License

This project is part of a graduation project for elderly care monitoring system.
