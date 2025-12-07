import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "Physical AI & Humanoid Robotics",
  description: "Bridging the gap between the digital brain and the physical body",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}

