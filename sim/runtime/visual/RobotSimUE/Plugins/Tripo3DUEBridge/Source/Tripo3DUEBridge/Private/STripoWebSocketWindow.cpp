#include "STripoWebSocketWindow.h"
#include "TripoVersionCompat.h"
#include "TripoWebSocketServer.h"
#include "TripoProtocol.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Views/SListView.h"

#define LOCTEXT_NAMESPACE "TripoWebSocketWindow"

// ————— Lifecycle —————

void STripoWebSocketWindow::Construct(const FArguments& InArgs)
{
	bIsServerRunning = false;
	bIsClientConnected = false;

	// Bind to server events
	FTripoWebSocketServer::Get().OnConnectionStatusChanged.AddRaw(this, &STripoWebSocketWindow::OnConnectionStatusChanged);
	FTripoWebSocketServer::Get().OnMessageReceived.AddRaw(this, &STripoWebSocketWindow::OnMessageReceived);

	ChildSlot
	[
		SNew(SBorder)
		.BorderImage(FAppStyle::Get().GetBrush("ToolPanel.GroupBorder"))
		.Padding(8.0f)
		[
			SNew(SVerticalBox)

			// Title
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0, 0, 0, 8)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("WindowTitle", "Tripo Bridge"))
				.Font(FAppStyle::Get().GetFontStyle("HeadingExtraSmall"))
				.Justification(ETextJustify::Center)
			]

			// Server Status Section
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0, 0, 0, 8)
			[
				SNew(SBorder)
				.BorderImage(FAppStyle::Get().GetBrush("ToolPanel.DarkGroupBorder"))
				.Padding(8.0f)
				[
					SNew(SVerticalBox)

					+ SVerticalBox::Slot()
					.AutoHeight()
					.Padding(0, 0, 0, 4)
					[
						SNew(SHorizontalBox)

						+ SHorizontalBox::Slot()
						.AutoWidth()
						.VAlign(VAlign_Center)
						[
							SNew(STextBlock)
							.Text(LOCTEXT("ServerStatusLabel", "Server Status: "))
						]

						+ SHorizontalBox::Slot()
						.FillWidth(1.0f)
						.VAlign(VAlign_Center)
						[
							SNew(STextBlock)
							.Text(this, &STripoWebSocketWindow::GetServerStatusText)
							.ColorAndOpacity(this, &STripoWebSocketWindow::GetServerStatusColor)
						]
					]

					+ SVerticalBox::Slot()
					.AutoHeight()
					[
						SNew(STextBlock)
						.Text(this, &STripoWebSocketWindow::GetConnectionInfoText)
						.Font(FAppStyle::Get().GetFontStyle("SmallFont"))
					]

					+ SVerticalBox::Slot()
					.AutoHeight()
					.Padding(0, 4, 0, 0)
					[
						SNew(STextBlock)
						.Text(FText::Format(LOCTEXT("PortLabel", "Port: {0}"), FText::AsNumber(TripoProtocol::SERVER_PORT)))
						.Font(FAppStyle::Get().GetFontStyle("SmallFont"))
						.ColorAndOpacity(FSlateColor::UseSubduedForeground())
					]
				]
			]

			// Control Button
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0, 0, 0, 8)
			[
				SNew(SButton)
				.Text(this, &STripoWebSocketWindow::GetServerButtonText)
				.HAlign(HAlign_Center)
				.ButtonColorAndOpacity(this, &STripoWebSocketWindow::GetServerButtonColor)
				.OnClicked(this, &STripoWebSocketWindow::OnToggleServerClicked)
			]

			// Message Log Section
			+ SVerticalBox::Slot()
			.FillHeight(1.0f)
			[
				SNew(SBorder)
				.BorderImage(FAppStyle::Get().GetBrush("ToolPanel.DarkGroupBorder"))
				.Padding(8.0f)
				[
					SNew(SVerticalBox)

					+ SVerticalBox::Slot()
					.AutoHeight()
					.Padding(0, 0, 0, 4)
					[
						SNew(SHorizontalBox)

						+ SHorizontalBox::Slot()
						.FillWidth(1.0f)
						[
							SNew(STextBlock)
							.Text(LOCTEXT("MessageLogLabel", "Message Log"))
							.Font(FAppStyle::Get().GetFontStyle("SmallFont"))
						]

						+ SHorizontalBox::Slot()
						.AutoWidth()
						[
							SNew(SButton)
							.Text(LOCTEXT("ClearButton", "Clear"))
							.OnClicked(this, &STripoWebSocketWindow::OnClearLogClicked)
						]
					]

					+ SVerticalBox::Slot()
					.FillHeight(1.0f)
					[
						SAssignNew(MessageLogListView, SListView<TSharedPtr<FString>>)
						.ListItemsSource(&MessageLogItems)
						.OnGenerateRow_Lambda([](TSharedPtr<FString> Item, const TSharedRef<STableViewBase>& OwnerTable)
						{
							return SNew(STableRow<TSharedPtr<FString>>, OwnerTable)
							[
								SNew(STextBlock)
								.Text(FText::FromString(*Item))
								.Font(FAppStyle::Get().GetFontStyle("SmallFont"))
							];
						})
					]
				]
			]
		]
	];

	// Auto-start server when window opens
	if (!FTripoWebSocketServer::Get().IsServerRunning())
	{
		FTripoWebSocketServer::Get().StartServer();
		bIsServerRunning = true;
		OnMessageReceived(TEXT("Server started automatically"));
	}
}

STripoWebSocketWindow::~STripoWebSocketWindow()
{
	// Unbind from server events
	FTripoWebSocketServer::Get().OnConnectionStatusChanged.RemoveAll(this);
	FTripoWebSocketServer::Get().OnMessageReceived.RemoveAll(this);

	// Stop server when window closes
	if (FTripoWebSocketServer::Get().IsServerRunning())
	{
		FTripoWebSocketServer::Get().StopServer();
	}
}

// ————— Event Handlers —————

FReply STripoWebSocketWindow::OnToggleServerClicked()
{
	if (bIsServerRunning)
	{
		// Stop server
		FTripoWebSocketServer::Get().StopServer();
		bIsServerRunning = false;
		bIsClientConnected = false;
		ConnectedClientInfo.Empty();
		OnMessageReceived(TEXT("Server stopped"));
	}
	else
	{
		// Start server
		if (FTripoWebSocketServer::Get().StartServer())
		{
			bIsServerRunning = true;
			OnMessageReceived(TEXT("Server started"));
		}
		else
		{
			OnMessageReceived(TEXT("Failed to start server"));
		}
	}

	return FReply::Handled();
}

FReply STripoWebSocketWindow::OnClearLogClicked()
{
	MessageLogItems.Empty();
	RefreshMessageLog();

	return FReply::Handled();
}

// ————— Internal —————

FText STripoWebSocketWindow::GetServerStatusText() const
{
	if (bIsServerRunning)
	{
		return bIsClientConnected ?
			LOCTEXT("StatusConnected", "Connected") :
			LOCTEXT("StatusListening", "Listening...");
	}
	return LOCTEXT("StatusStopped", "Stopped");
}

FSlateColor STripoWebSocketWindow::GetServerStatusColor() const
{
	if (bIsServerRunning)
	{
		return bIsClientConnected ?
			FSlateColor(FLinearColor::Green) :
			FSlateColor(FLinearColor::Yellow);
	}
	return FSlateColor(FLinearColor::Red);
}

FText STripoWebSocketWindow::GetConnectionInfoText() const
{
	if (bIsClientConnected)
	{
		return FText::Format(LOCTEXT("ClientConnectedInfo", "Client: {0}"), FText::FromString(ConnectedClientInfo));
	}
	else if (bIsServerRunning)
	{
		return FText::Format(LOCTEXT("WaitingForConnection", "Waiting for connection on {0}:{1}"),
			FText::FromString(TripoProtocol::SERVER_HOST),
			FText::AsNumber(TripoProtocol::SERVER_PORT));
	}
	return LOCTEXT("ServerNotRunning", "Server not running");
}

FText STripoWebSocketWindow::GetServerButtonText() const
{
	return bIsServerRunning ?
		LOCTEXT("StopServerButton", "Stop Server") :
		LOCTEXT("StartServerButton", "Start Server");
}

FSlateColor STripoWebSocketWindow::GetServerButtonColor() const
{
	return bIsServerRunning ?
		FSlateColor(FLinearColor(0.5f, 0.5f, 0.5f)) : // Gray when running
		FSlateColor(FLinearColor(0.0f, 0.8f, 0.0f)); // Green when stopped
}

void STripoWebSocketWindow::OnConnectionStatusChanged(bool bIsConnected, const FString& ClientInfo)
{
	bIsClientConnected = bIsConnected;
	ConnectedClientInfo = ClientInfo;

	if (bIsConnected)
	{
		OnMessageReceived(FString::Printf(TEXT("Client connected: %s"), *ClientInfo));
	}
	else
	{
		OnMessageReceived(TEXT("Client disconnected"));
	}
}

void STripoWebSocketWindow::OnMessageReceived(const FString& Message)
{
	// Add timestamp
	FDateTime Now = FDateTime::Now();
	FString TimestampedMessage = FString::Printf(TEXT("[%02d:%02d:%02d] %s"),
		Now.GetHour(), Now.GetMinute(), Now.GetSecond(), *Message);

	MessageLogItems.Add(MakeShared<FString>(TimestampedMessage));

	// Limit log size
	if (MessageLogItems.Num() > MaxLogMessages)
	{
		MessageLogItems.RemoveAt(0);
	}

	RefreshMessageLog();
}

void STripoWebSocketWindow::RefreshMessageLog()
{
	if (MessageLogListView.IsValid())
	{
		MessageLogListView->RequestListRefresh();

		// Scroll to the last item (newest message)
		if (MessageLogItems.Num() > 0)
		{
			MessageLogListView->RequestScrollIntoView(MessageLogItems.Last());
		}
	}
}

#undef LOCTEXT_NAMESPACE
