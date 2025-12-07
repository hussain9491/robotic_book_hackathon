import './globals.css';

export const metadata = {
  title: 'Speckit Robotics',
  description: 'Physical AI & Humanoid Robotics',
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
