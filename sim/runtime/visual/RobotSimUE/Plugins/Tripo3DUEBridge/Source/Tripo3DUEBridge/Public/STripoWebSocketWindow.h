// Copyright Tripo3D. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/DeclarativeSyntaxSupport.h"

/**
 * Slate widget for Tripo3D WebSocket server UI
 * Displays connection status, controls, and message log
 */
class TRIPO3DUEBRIDGE_API STripoWebSocketWindow : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(STripoWebSocketWindow) {}
	SLATE_END_ARGS()

	/** Constructs this widget with InArgs */
	void Construct(const FArguments& InArgs);

	/** Destructor */
	virtual ~STripoWebSocketWindow();

private:
	/** Handle toggle server button clicked */
	FReply OnToggleServerClicked();

	/** Handle clear log button clicked */
	FReply OnClearLogClicked();

	/** Get server button text */
	FText GetServerButtonText() const;

	/** Get server button color */
	FSlateColor GetServerButtonColor() const;

	/** Get server status text */
	FText GetServerStatusText() const;

	/** Get server status color */
	FSlateColor GetServerStatusColor() const;

	/** Get connection info text */
	FText GetConnectionInfoText() const;

	/** Handle connection status changed */
	void OnConnectionStatusChanged(bool bIsConnected, const FString& ClientInfo);

	/** Handle message received */
	void OnMessageReceived(const FString& Message);

	/** Update message log widget */
	void RefreshMessageLog();

private:
	/** Cached server status */
	bool bIsServerRunning;

	/** Cached connection status */
	bool bIsClientConnected;

	/** Cached client info */
	FString ConnectedClientInfo;

	/** Message log items */
	TArray<TSharedPtr<FString>> MessageLogItems;

	/** Message log list view widget */
	TSharedPtr<class SListView<TSharedPtr<FString>>> MessageLogListView;

	/** Maximum log messages to keep */
	static const int32 MaxLogMessages = 50;
};
